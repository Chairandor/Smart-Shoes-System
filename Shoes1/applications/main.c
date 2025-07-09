#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <finsh.h>
#include <string.h>  // 添加头文件解决memset/strstr/strncpy警告
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

/* 新增GPS模块配置 */
#define GPS_UART_NAME "uart2"
#define GPS_BAUD_RATE 9600
#define GPS_BUF_SIZE 128
#define GPS_RECV_TIMEOUT 10


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
        rt_uint32_t first_peak[3];
        rt_bool_t is_measuring;
        rt_tick_t measure_start;
        rt_uint32_t measure_period;
    } step;

    /* 蓝牙 */
    rt_device_t ble_dev;
    rt_sem_t ble_rx_sem;
    char ble_rx_buf[BLE_BUF_SIZE];
    rt_bool_t ble_continuous_output;
    rt_uint32_t ble_last_active; // 记录最后活动时间
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

/* 自定义字符串转整数函数 */
static int custom_atoi(const char *str) {
    int result = 0, sign = 1;
    while (*str == ' ' || *str == '\t') str++;
    if (*str == '-') { sign = -1; str++; }
    else if (*str == '+') str++;
    while (*str >= '0' && *str <= '9')
        result = result * 10 + (*str++ - '0');
    return sign * result;
}

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
                if (sensor.ble_rx_buf[0] == 'C') {
                    sensor.ble_continuous_output = !sensor.ble_continuous_output;
                    char resp[32];
                    rt_snprintf(resp, sizeof(resp), "Mode:%s\n",
                               sensor.ble_continuous_output ? "RT-ON" : "RT-OFF");
                    rt_device_write(sensor.ble_dev, 0, resp, rt_strlen(resp));
                }
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

            /* 测量结束处理 */
            if (sensor.step.is_measuring &&
                rt_tick_get() - sensor.step.measure_start >=
                rt_tick_from_millisecond(sensor.step.measure_period))
            {
                sensor.step.is_measuring = RT_FALSE;
                rt_uint32_t cadence = (sensor.step.measure_period > 0) ?
                    sensor.step.step_count * 60000 / sensor.step.measure_period : 0;

                /* 找出主要着地区域 */
                int max_idx = 0;
                for (int i = 1; i < 3; i++)
                    if (sensor.step.first_peak[i] > sensor.step.first_peak[max_idx])
                        max_idx = i;

                const char *parts[] = {"Heel", "Outboard", "Ball"};
                rt_kprintf("\nResults: %ds\nSteps: %d\nCadence: %d/min\n"
                          "First contact: %s (%d times)\n",
                    sensor.step.measure_period/1000, sensor.step.step_count, cadence,
                    parts[max_idx], sensor.step.first_peak[max_idx]);

                /* 重置计数器 */
                sensor.step.step_count = 0;
                for (int i = 0; i < 3; i++) sensor.step.first_peak[i] = 0;
            }
        }
        rt_thread_mdelay(1);
    }
}

/* 持续输出压力值命令 */
static void start_continuous_output(int argc, char **argv) {
    sensor.continuous_output = RT_TRUE;
    rt_kprintf("Continuous output ON\n");
}
MSH_CMD_EXPORT(start_continuous_output, Start pressure output);

static void stop_continuous_output(int argc, char **argv) {
    sensor.continuous_output = RT_FALSE;
    rt_kprintf("Continuous output OFF\n");
}
MSH_CMD_EXPORT(stop_continuous_output, Stop pressure output);

static void start_step_measurement(int argc, char **argv) {
    if (argc != 2) {
        rt_kprintf("Usage: measure <seconds>\n");
        return;
    }

    int sec = custom_atoi(argv[1]);
    if (sec <= 0) {
        rt_kprintf("Invalid time\n");
        return;
    }

    sensor.step.measure_period = sec * 1000;
    sensor.step.measure_start = rt_tick_get();
    sensor.step.is_measuring = RT_TRUE;
    sensor.step.step_count = 0;
    for (int i = 0; i < 3; i++) sensor.step.first_peak[i] = 0;
    rt_kprintf("Measuring for %d seconds...\n", sec);
}
MSH_CMD_EXPORT(start_step_measurement, measure <seconds> - Start step measurement);

/* 查看蓝牙状态 */
static void ble_status(int argc, char **argv) {
    rt_kprintf("BLE Status:\n");
    rt_kprintf("  Continuous output: %s\n",
              sensor.ble_continuous_output ? "ON" : "OFF");
    rt_kprintf("  Connection: %s\n",
              sensor.ble_dev ? "Active" : "Inactive");
}
MSH_CMD_EXPORT(ble_status, Show BLE status);

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

/* 替换strsep的自定义分割函数 */
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

/* 打印GPS信息命令 */
static void gps_info(int argc, char **argv) {
    if (gps.data_valid) {
        rt_kprintf("UTC Time: %s\n", gps.utc_time);
        rt_kprintf("Latitude: %s %c\n", gps.latitude, gps.ns);
        rt_kprintf("Longitude: %s %c\n", gps.longitude, gps.ew);
    } else {
        rt_kprintf("No valid GPS data\n");
    }
}
MSH_CMD_EXPORT(gps_info, Print GPS information);

/* 新增GPS路程计算相关变量 */
struct {
    rt_tick_t start_time;
    rt_bool_t is_calculating;
    double last_lat;     // 上一次纬度(度)
    double last_lon;     // 上一次经度(度)
    double total_distance; // 总路程(米)
    rt_thread_t calc_thread;
} gps_distance;

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

                // 调试输出
                // rt_kprintf("Segment: %.2fm, Total: %.2fm\n",
                //           segment, gps_distance.total_distance);
            }

            /* 更新上一个点坐标 */
            gps_distance.last_lat = current_lat;
            gps_distance.last_lon = current_lon;
            first_point = RT_FALSE;
        }

        /* 每秒更新一次 */
        rt_thread_mdelay(1000);

        /* 检查是否超时 */
        if (rt_tick_get() - gps_distance.start_time > rt_tick_from_millisecond(60000)) {
            gps_distance.is_calculating = RT_FALSE;
        }
    }
}

/* 开始计算路程命令 */
static void start_distance_calculation(int argc, char **argv) {
    if (argc != 2) {
        rt_kprintf("Usage: distance <seconds>\n");
        rt_kprintf("Note: Max time is 60 seconds\n");
        return;
    }

    int sec = custom_atoi(argv[1]);
    if (sec <= 0 || sec > 60) {
        rt_kprintf("Invalid time (1-60 seconds)\n");
        return;
    }

    if (gps_distance.is_calculating) {
        rt_kprintf("Calculation is already in progress\n");
        return;
    }

    /* 等待获取第一个有效GPS点 */
    rt_kprintf("Waiting for valid GPS data...\n");
    while (!gps.data_valid) {
        rt_thread_mdelay(100);
    }

    /* 初始化计算 */
    gps_distance.start_time = rt_tick_get();
    gps_distance.is_calculating = RT_TRUE;
    gps_distance.total_distance = 0.0;

    /* 创建计算线程 */
    gps_distance.calc_thread = rt_thread_create(
        "gps_dist", gps_distance_thread_entry, RT_NULL,
        512, RT_THREAD_PRIORITY_MAX-4, 20);

    if (gps_distance.calc_thread) {
        rt_thread_startup(gps_distance.calc_thread);
        rt_kprintf("Calculating distance for %d seconds...\n", sec);

        /* 等待计算完成 */
        while (gps_distance.is_calculating &&
               rt_tick_get() - gps_distance.start_time < rt_tick_from_millisecond(sec*1000)) {
            rt_thread_mdelay(100);
        }

        /* 停止计算 */
        gps_distance.is_calculating = RT_FALSE;
        rt_thread_mdelay(100); // 确保线程退出

        /* 打印结果 */
        // 将总距离拆分为整数和小数部分
        int integer_part = (int)gps_distance.total_distance;
        int decimal_part = (int)((gps_distance.total_distance - integer_part) * 100); // 保留2位小数
        rt_kprintf("Total distance: %d.%02d meters\n", integer_part, decimal_part);
    } else {
        rt_kprintf("Failed to create calculation thread\n");
    }
}
MSH_CMD_EXPORT(start_distance_calculation, distance <seconds> - Calculate moving distance in specified time);

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
    }

    return RT_EOK;
}
INIT_APP_EXPORT(sensor_init);

/* main函数 */
int main(void) {
    rt_kprintf("Smart Shoes System Ready\n");
    return 0;
}
