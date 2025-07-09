#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <finsh.h>
#include <string.h>
#include <math.h>

#define M_PI 3.14159265358979323846

/* 硬件配置 */
/* 压力传感器 */
#define SENSOR_PIN0 GET_PIN(A, 0)  // PA0 -> ADC1_IN0
#define SENSOR_PIN1 GET_PIN(A, 1)  // PA1 -> ADC1_IN1
#define SENSOR_PIN2 GET_PIN(A, 4)  // PA4 -> ADC1_IN4
/* 蓝牙 */
#define BLE_UART_NAME "uart3"
#define BLE_BAUD_RATE 9600
#define BLE_BUF_SIZE 64
#define BLE_SEND_INTERVAL_MS 200  // 连续发送间隔

/* GPS模块配置 */
#define GPS_UART_NAME "uart2"
#define GPS_BAUD_RATE 9600
#define GPS_BUF_SIZE 128
#define GPS_RECV_TIMEOUT 10

/* 测量参数 */
#define MEASURE_INTERVAL_MS (60 * 1000)    // 每分钟测量一次
#define REPORT_INTERVAL_MS (5 * 60 * 1000) // 每5分钟报告一次
#define MAX_MEASUREMENTS 5                 // 5次测量

/* 全局变量 */
static struct {
    /* 压力传感器 */
    rt_adc_device_t dev[3];
    int values[3];
    rt_mutex_t lock;
    rt_bool_t continuous_output;

    /* 步频测量 */
    struct {
        rt_tick_t last_step_time;
        rt_uint32_t step_count;
        rt_uint32_t first_peak[3]; // 0:Heel, 1:Outboard, 2:Ball
        rt_bool_t is_measuring;
        rt_tick_t measure_start;
        rt_uint32_t measure_period;
    } step;

    /* 蓝牙 */
    rt_device_t ble_dev;
    rt_sem_t ble_rx_sem;
    char ble_rx_buf[BLE_BUF_SIZE];
    rt_bool_t ble_continuous_output;
    rt_uint32_t ble_last_active;

    /* 新增测量控制 */
    rt_bool_t measurement_enabled;
    rt_uint32_t measurement_count;
    struct {
        rt_uint32_t steps;
        rt_uint32_t first_contact[3]; // 各部位首次接触次数
        double distance;
    } measurements[MAX_MEASUREMENTS];
} sensor;

/* GPS数据结构 */
struct {
    char raw_data[GPS_BUF_SIZE];
    char utc_time[12];       // UTC时间 HHMMSS.SSS
    char latitude[12];       // 纬度 DDMM.MMMMM
    char ns;                 // N/S
    char longitude[13];      // 经度 DDDMM.MMMMM
    char ew;                 // E/W
    rt_sem_t data_sem;
    rt_device_t dev;
    rt_bool_t data_valid;
} gps;

/* 路程计算结构 */
struct {
    rt_tick_t start_time;
    rt_bool_t is_calculating;
    double last_lat;         // 上一次纬度(度)
    double last_lon;         // 上一次经度(度)
    double total_distance;   // 总路程(米)
    rt_thread_t calc_thread;
} gps_distance;

/* 简单平方根近似 */
static double simple_sqrt(double val) {
    double root = val/2;
    for(int i=0; i<10; i++) {
        root = (root + val/root)/2;
    }
    return root;
}

/* ADC原始值转物理量 */
static int adc_to_pressure(rt_uint32_t raw) {
    float voltage = raw * 3.3f / 4096;
    float term1 = (voltage * 10 - 1) / 100.0;
    float val = -647320 * term1 * term1 + 223207 * term1 + 1038.6;
    return (val < 250) ? 0 : (int)val;
}

/* 检测步态 */
static void detect_step(void) {
    static int prev_values[3] = {0};
    static rt_uint8_t peak_order = 0, peak_flags = 0;

    for (int i = 0; i < 3; i++) {
        if (sensor.values[i] > prev_values[i] + 100 && !(peak_flags & (1 << i))) {
            peak_order = (peak_order << 2) | i;
            peak_flags |= (1 << i);
        }
        prev_values[i] = sensor.values[i];
    }

    if (peak_flags == 0x07) {
        rt_tick_t current = rt_tick_get();
        if (sensor.step.is_measuring) {
            sensor.step.step_count++;
            rt_uint8_t first_part = peak_order >> 4;
            if (first_part < 3) sensor.step.first_peak[first_part]++;
        }
        sensor.step.last_step_time = current;
        peak_order = peak_flags = 0;
    }
}

/* 蓝牙接收回调 */
static rt_err_t ble_rx_ind(rt_device_t dev, rt_size_t size) {
    rt_sem_release(sensor.ble_rx_sem);
    return RT_EOK;
}

/* 处理蓝牙命令 */
static void process_ble_command(char cmd) {
    switch (cmd) {
    case 'I': // 开始/停止测量
        sensor.measurement_enabled = !sensor.measurement_enabled;

        if (sensor.measurement_enabled) {
            // 开始测量
            sensor.measurement_count = 0;
            rt_memset(sensor.measurements, 0, sizeof(sensor.measurements));
            rt_kprintf("[BLE] Measurement started. Will report every 5 minutes.\n");

            // 发送确认消息
            char resp[] = "Measurement started\n";
            rt_device_write(sensor.ble_dev, 0, resp, sizeof(resp)-1);
        } else {
            // 停止测量
            rt_kprintf("[BLE] Measurement stopped.\n");

            // 发送确认消息
            char resp[] = "Measurement stopped\n";
            rt_device_write(sensor.ble_dev, 0, resp, sizeof(resp)-1);
        }
        break;

    case 'C': // 切换连续输出模式
        sensor.ble_continuous_output = !sensor.ble_continuous_output;
        char resp[32];
        rt_snprintf(resp, sizeof(resp), "Mode:%s\n",
                   sensor.ble_continuous_output ? "RT-ON" : "RT-OFF");
        rt_device_write(sensor.ble_dev, 0, resp, rt_strlen(resp));
        break;
    }
}

/* 蓝牙线程 */
static void ble_thread_entry(void* param) {
    /* 初始化 */
    sensor.ble_rx_sem = rt_sem_create("ble_rx", 0, RT_IPC_FLAG_FIFO);

    sensor.ble_dev = rt_device_find(BLE_UART_NAME);
    if (!sensor.ble_dev) {
        rt_kprintf("[BLE] Device not found\n");
        return;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BLE_BAUD_RATE;
    rt_device_control(sensor.ble_dev, RT_DEVICE_CTRL_CONFIG, &config);

    if (rt_device_open(sensor.ble_dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK) {
        rt_kprintf("[BLE] Open failed\n");
        return;
    }
    rt_device_set_rx_indicate(sensor.ble_dev, ble_rx_ind);
    rt_kprintf("[BLE] Ready\n");

    sensor.ble_last_active = rt_tick_get();

    /* 主循环 */
    while (1) {
        /* 非阻塞检查命令 */
        if (rt_sem_take(sensor.ble_rx_sem, 0) == RT_EOK) {
            rt_size_t len = rt_device_read(sensor.ble_dev, 0, sensor.ble_rx_buf, BLE_BUF_SIZE);
            if (len > 0) {
                sensor.ble_last_active = rt_tick_get();
                process_ble_command(sensor.ble_rx_buf[0]);
            }
        }

        /* 连续输出 */
        if (sensor.ble_continuous_output) {
            char msg[64];
            rt_mutex_take(sensor.lock, RT_WAITING_FOREVER);
            int n = rt_snprintf(msg, sizeof(msg), "Heel:%d Outboard:%d Ball:%d\n",
                sensor.values[0], sensor.values[1],
                sensor.values[2]);
            rt_mutex_release(sensor.lock);
            rt_device_write(sensor.ble_dev, 0, msg, n);
        }

        rt_thread_mdelay(BLE_SEND_INTERVAL_MS);
    }
}

/* 执行一分钟测量 */
static void perform_one_minute_measurement(void) {
    if (sensor.measurement_count >= MAX_MEASUREMENTS) {
        return;
    }

    // 启动步态测量
    sensor.step.measure_period = 60 * 1000; // 1分钟
    sensor.step.measure_start = rt_tick_get();
    sensor.step.is_measuring = RT_TRUE;
    sensor.step.step_count = 0;
    for (int i = 0; i < 3; i++) sensor.step.first_peak[i] = 0;

    // 启动GPS距离计算
    gps_distance.start_time = rt_tick_get();
    gps_distance.is_calculating = RT_TRUE;
    gps_distance.total_distance = 0.0;

    // 等待测量完成
    while (rt_tick_get() - sensor.step.measure_start < rt_tick_from_millisecond(sensor.step.measure_period)) {
        rt_thread_mdelay(100);
    }

    // 停止测量
    sensor.step.is_measuring = RT_FALSE;
    gps_distance.is_calculating = RT_FALSE;
    rt_thread_mdelay(100); // 确保线程退出

    // 保存测量结果
    int idx = sensor.measurement_count;
    sensor.measurements[idx].steps = sensor.step.step_count;

    // 找出主要着地区域
    int max_idx = 0;
    for (int i = 1; i < 3; i++) {
        if (sensor.step.first_peak[i] > sensor.step.first_peak[max_idx]) {
            max_idx = i;
        }
    }

    // 记录各部位首次接触次数
    for (int i = 0; i < 3; i++) {
        sensor.measurements[idx].first_contact[i] = sensor.step.first_peak[i];
    }

    sensor.measurements[idx].distance = gps_distance.total_distance;

    sensor.measurement_count++;

    // 修改输出格式，确保浮点数正确显示
    int integer_part = (int)gps_distance.total_distance;
    int decimal_part = (int)((gps_distance.total_distance - integer_part) * 100); // 保留2位小数
    rt_kprintf("[Measurement %d/%d] Steps: %d, Distance: %d.%02dm\n",
              sensor.measurement_count, MAX_MEASUREMENTS,
              sensor.measurements[idx].steps,
              integer_part, decimal_part);
}

/* 生成并发送5分钟报告 */
static void send_five_minute_report(void) {
    if (sensor.measurement_count == 0) {
        return;
    }

    // 计算统计数据
    rt_uint32_t total_steps = 0;
    double total_distance = 0.0;
    rt_uint32_t contact_counts[3] = {0};

    for (int i = 0; i < sensor.measurement_count; i++) {
        total_steps += sensor.measurements[i].steps;
        total_distance += sensor.measurements[i].distance;

        for (int j = 0; j < 3; j++) {
            contact_counts[j] += sensor.measurements[i].first_contact[j];
        }
    }

    // 计算平均步频(步/分钟)
    double avg_cadence = (double)total_steps / (sensor.measurement_count * 1.0);

    // 计算平均步长(米/步)
    double avg_stride = (total_steps > 0) ? (total_distance / total_steps) : 0.0;

    // 找出主要着地区域
    int max_idx = 0;
    for (int i = 1; i < 3; i++) {
        if (contact_counts[i] > contact_counts[max_idx]) {
            max_idx = i;
        }
    }
    const char* contact_parts[] = {"Heel", "Outboard", "Ball"};

    // 将浮点数转换为整数和小数部分
    int total_dist_int = (int)total_distance;
    int total_dist_dec = (int)((total_distance - total_dist_int) * 100);

    int avg_stride_int = (int)avg_stride;
    int avg_stride_dec = (int)((avg_stride - avg_stride_int) * 100);

    // 生成报告
    char report[256];
    int len = rt_snprintf(report, sizeof(report),
        "=== 5-Minute Report ===\n"
        "Measurements: %d\n"
        "Total Steps: %d\n"
        "Avg Cadence: %d.%d steps/min\n"
        "Primary Contact: %s (%d times)\n"
        "Total Distance: %d.%02d m\n"
        "Avg Stride: %d.%02d m/step\n"
        "=====================\n",
        sensor.measurement_count,
        total_steps,
        (int)avg_cadence, (int)((avg_cadence - (int)avg_cadence) * 10),
        contact_parts[max_idx],
        contact_counts[max_idx],
        total_dist_int, total_dist_dec,
        avg_stride_int, avg_stride_dec);

    // 发送报告
    rt_device_write(sensor.ble_dev, 0, report, len);
    rt_kprintf("%s", report);

    // 重置计数器
    sensor.measurement_count = 0;
}

/* 测量控制线程 */
static void measurement_thread_entry(void* param) {
    rt_tick_t last_measure_time = 0;
    rt_tick_t last_report_time = 0;

    while (1) {
        rt_thread_mdelay(1000); // 每秒检查一次

        if (!sensor.measurement_enabled) {
            continue;
        }

        rt_tick_t now = rt_tick_get();

        // 每分钟执行一次测量
        if (now - last_measure_time >= rt_tick_from_millisecond(MEASURE_INTERVAL_MS)) {
            perform_one_minute_measurement();
            last_measure_time = now;
        }

        // 每5分钟生成一次报告
        if (now - last_report_time >= rt_tick_from_millisecond(REPORT_INTERVAL_MS)) {
            send_five_minute_report();
            last_report_time = now;
        }
    }
}

/* 压力传感器线程 */
static void sensor_thread_entry(void *param) {
    rt_uint32_t raw[3];
    int sum[3] = {0}, count = 0;

    while (1) {
        /* 采样 */
        raw[0] = rt_adc_read(sensor.dev[0], 0);
        raw[1] = rt_adc_read(sensor.dev[1], 1);
        raw[2] = rt_adc_read(sensor.dev[2], 4);

        /* 滑动平均 */
        for (int i = 0; i < 3; i++) sum[i] += raw[i];
        if (++count >= 10) {
            rt_mutex_take(sensor.lock, RT_WAITING_FOREVER);
            for (int i = 0; i < 3; i++) {
                sensor.values[i] = adc_to_pressure(sum[i] / 10);
                sum[i] = 0;
            }
            rt_mutex_release(sensor.lock);
            count = 0;

            /* 连续输出模式 */
            if (sensor.continuous_output) {
                rt_kprintf("Heel: %d, Outboard: %d, Ball: %d\n",
                    sensor.values[0], sensor.values[1], sensor.values[2]);
            }

            /* 步态检测 */
            detect_step();
        }
        rt_thread_mdelay(1);
    }
}

/* GPS初始化函数 */
static int gps_init(void) {
    /* 查找串口设备 */
    gps.dev = rt_device_find(GPS_UART_NAME);
    if (!gps.dev) {
        rt_kprintf("[GPS] Device not found\n");
        return -RT_ERROR;
    }

    /* 配置串口 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = GPS_BAUD_RATE;
    rt_device_control(gps.dev, RT_DEVICE_CTRL_CONFIG, &config);

    /* 打开设备 */
    if (rt_device_open(gps.dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK) {
        rt_kprintf("[GPS] Open failed\n");
        return -RT_ERROR;
    }

    /* 创建信号量 */
    gps.data_sem = rt_sem_create("gps_data", 0, RT_IPC_FLAG_FIFO);
    if (!gps.data_sem) {
        rt_kprintf("[GPS] Sem create failed\n");
        return -RT_ERROR;
    }

    /* 初始化状态 */
    gps.data_valid = RT_FALSE;
    memset(gps.raw_data, 0, sizeof(gps.raw_data));
    memset(gps.utc_time, 0, sizeof(gps.utc_time));
    memset(gps.latitude, 0, sizeof(gps.latitude));
    memset(gps.longitude, 0, sizeof(gps.longitude));
    gps.ns = ' ';
    gps.ew = ' ';

    rt_kprintf("[GPS] Initialized\n");
    return RT_EOK;
}

/* GPS接收回调函数 */
static rt_err_t gps_rx_ind(rt_device_t dev, rt_size_t size) {
    static char buffer[GPS_BUF_SIZE];
    static size_t index = 0;
    char ch;

    while (rt_device_read(dev, -1, &ch, 1) == 1) {
        /* 处理换行符 */
        if (ch == '\n' || index >= sizeof(buffer) - 1) {
            buffer[index] = '\0';

            /* 检查是否为GNRMC/GPRMC消息 */
            if (strstr(buffer, "$GNRMC") || strstr(buffer, "$GPRMC")) {
                strncpy(gps.raw_data, buffer, sizeof(gps.raw_data));
                rt_sem_release(gps.data_sem);
            }

            index = 0;
        }
        /* 处理回车符 */
        else if (ch == '\r') {
            // 忽略回车符
        }
        /* 存储有效字符 */
        else {
            buffer[index++] = ch;
        }
    }
    return RT_EOK;
}

/* 自定义分割函数 */
static char* custom_strsep(char** str, const char* delim) {
    char* start = *str;
    char* p = *str;

    if (!*str || !**str) return NULL;

    while (*p) {
        if (strchr(delim, *p)) {
            *p = '\0';
            *str = p + 1;
            return start;
        }
        p++;
    }

    *str = NULL;
    return start;
}

/* GPS数据处理线程 */
static void gps_thread_entry(void* param) {
    char *token, *ptr;
    int field_count;

    /* 设置接收回调 */
    rt_device_set_rx_indicate(gps.dev, gps_rx_ind);

    while (1) {
        /* 等待数据信号量 */
        if (rt_sem_take(gps.data_sem, RT_WAITING_FOREVER) == RT_EOK) {
            gps.data_valid = RT_FALSE;
            field_count = 0;
            ptr = gps.raw_data;

            /* 解析逗号分隔的字段 */
            while ((token = custom_strsep(&ptr, ",")) != NULL) {
                switch (field_count) {
                case 1:  // UTC时间
                    strncpy(gps.utc_time, token, sizeof(gps.utc_time)-1);
                    break;
                case 2:  // 定位状态
                    gps.data_valid = (*token == 'A');
                    break;
                case 3:  // 纬度
                    strncpy(gps.latitude, token, sizeof(gps.latitude)-1);
                    break;
                case 4:  // N/S
                    if (*token == 'N' || *token == 'S') gps.ns = *token;
                    break;
                case 5:  // 经度
                    strncpy(gps.longitude, token, sizeof(gps.longitude)-1);
                    break;
                case 6:  // E/W
                    if (*token == 'E' || *token == 'W') gps.ew = *token;
                    break;
                }
                field_count++;
            }
        }
    }
}

/* 将度分格式转换为十进制度数 */
static double dm_to_degrees(const char* dm, char hemisphere) {
    double degrees = 0.0;
    double minutes = 0.0;
    double decimal = 0.1;

    // 手动解析度分格式(DDMM.MMMMM)
    if (strlen(dm)){
        // 解析度部分
        degrees = 10*(dm[0]-'0') + (dm[1]-'0');

        // 解析分部分
        minutes = 10*(dm[2]-'0') + (dm[3]-'0');

        // 解析小数部分（如果有）
        if (dm[4] == '.') {
            for(int i=5; i<strlen(dm) && i<10; i++) {
                minutes += (dm[i]-'0') * decimal;
                decimal /= 10.0;
            }
        }
    }

    double result = degrees + minutes / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W') {
        result = -result;
    }
    return result;
}

/* 简化的距离计算 */
static double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    // 简化的平面近似计算（适用于小范围移动）
    const double R = 6371000.0 * (M_PI/180); // 每度的米数
    double x = (lon2 - lon1) * R * cos((lat1 + lat2)/2 * (M_PI/180));
    double y = (lat2 - lat1) * R;
    return simple_sqrt(x*x + y*y);
}

/* GPS路程计算线程 */
static void gps_distance_thread_entry(void* param) {
    double current_lat = 0.0, current_lon = 0.0;
    rt_bool_t first_point = RT_TRUE;

    gps_distance.total_distance = 0.0;

    while (gps_distance.is_calculating) {
        if (gps.data_valid) {
            /* 转换为十进制度数 */
            current_lat = dm_to_degrees(gps.latitude, gps.ns);
            current_lon = dm_to_degrees(gps.longitude, gps.ew);

            if (!first_point) {
                /* 计算与上一个点的距离并累加 */
                double segment = calculate_distance(
                    gps_distance.last_lat, gps_distance.last_lon,
                    current_lat, current_lon);

                gps_distance.total_distance += segment;
            }

            /* 更新上一个点坐标 */
            gps_distance.last_lat = current_lat;
            gps_distance.last_lon = current_lon;
            first_point = RT_FALSE;
        }

        /* 每秒更新一次 */
        rt_thread_mdelay(1000);
    }
}

/* 初始化 */
static int sensor_init(void) {
    /* 初始化互斥锁 */
    sensor.lock = rt_mutex_create("sens_lock", RT_IPC_FLAG_FIFO);

    /* 初始化ADC */
    if (!(sensor.dev[0] = (rt_adc_device_t)rt_device_find("adc1"))) {
        rt_kprintf("ADC1 not found\n");
        return -RT_ERROR;
    }
    sensor.dev[1] = sensor.dev[2] = sensor.dev[0];
    for (int i = 0; i < 3; i++) rt_adc_enable(sensor.dev[i], (i == 2) ? 4 : i);

    /* 初始化状态 */
    sensor.continuous_output = RT_FALSE;
    rt_memset(&sensor.step, 0, sizeof(sensor.step));
    sensor.measurement_enabled = RT_FALSE;
    sensor.measurement_count = 0;

    /* 创建传感器线程 */
    rt_thread_t sensor_tid = rt_thread_create(
        "sensor", sensor_thread_entry, RT_NULL,
        1024, RT_THREAD_PRIORITY_MAX-1, 20);
    if (!sensor_tid) {
        rt_kprintf("Create sensor thread failed\n");
        return -RT_ERROR;
    }
    rt_thread_startup(sensor_tid);

    /* 创建蓝牙线程 */
    rt_thread_t ble_tid = rt_thread_create(
        "ble", ble_thread_entry, RT_NULL,
        1024, RT_THREAD_PRIORITY_MAX-2, 20);
    if (ble_tid) rt_thread_startup(ble_tid);

    /* 初始化GPS */
    if (gps_init() == RT_EOK) {
        /* 创建GPS线程 */
        rt_thread_t gps_tid = rt_thread_create(
            "gps", gps_thread_entry, RT_NULL,
            1024, RT_THREAD_PRIORITY_MAX-3, 20);
        if (gps_tid) rt_thread_startup(gps_tid);

        /* 创建GPS距离计算线程 */
        gps_distance.calc_thread = rt_thread_create(
            "gps_dist", gps_distance_thread_entry, RT_NULL,
            512, RT_THREAD_PRIORITY_MAX-4, 20);
        if (gps_distance.calc_thread) {
            rt_thread_startup(gps_distance.calc_thread);
        }
    }

    /* 创建测量控制线程 */
    rt_thread_t measure_tid = rt_thread_create(
        "measure", measurement_thread_entry, RT_NULL,
        1024, RT_THREAD_PRIORITY_MAX-3, 20);
    if (measure_tid) rt_thread_startup(measure_tid);

    return RT_EOK;
}
INIT_APP_EXPORT(sensor_init);

/* main函数 */
int main(void) {
    rt_kprintf("Smart Shoes System Ready\n");
    rt_kprintf("Send 'I' via BLE to start/stop measurement\n");
    return 0;
}
