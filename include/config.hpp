/**
 * @file config.hpp
 * @brief ESP32-S3 PID温控项目配置文件
 * @author Claude Assistant
 * @date 2023-11-15
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Arduino.h>

// 硬件引脚定义
#define PIN_SDA                 5     // I2C SDA引脚 (GPIO5)
#define PIN_SCL                 6     // I2C SCL引脚 (GPIO6)
#define PIN_HEATER_PWM         10     // 加热器PWM输出引脚 (GPIO10)
#define PIN_ENCODER_A           1     // 旋转编码器A相 (GPIO1)
#define PIN_ENCODER_B           2     // 旋转编码器B相 (GPIO2)
#define PIN_ENCODER_BTN         3     // 旋转编码器按钮 (GPIO3)

// ADS1115配置
#define ADS_ADDRESS          0x48     // ADS1115 I2C地址
#define ADS_GAIN             GAIN_ONE // ±4.096V量程
#define ADS_SAMPLE_RATE      ADS1115_DR_128SPS // 128采样率

// NTC温度传感器参数
#define NTC_BETA             3950     // NTC Beta值
#define NTC_R0               100000.0f // 标称电阻值(100KΩ)
#define NTC_T0               25.0f    // 标称温度(摄氏度)
#define NTC_SERIES_R         100000.0f // 上拉电阻值(100KΩ)
#define REFERENCE_VOLTAGE    3.3f     // 参考电压

// PID控制器初始参数
#define PID_KP               10.0f    // 比例系数
#define PID_KI               0.4f     // 积分系数
#define PID_KD               40.0f    // 微分系数
#define PID_MIN_DERIV_TIME   2.0f     // 最小微分时间
#define HEATER_MAX_POWER     1.0f     // 加热器最大功率(0.0-1.0)

// 系统参数
#define SENSOR_TASK_STACK    8192     // 传感器任务堆栈大小
#define CONTROL_TASK_STACK   8192     // 控制任务堆栈大小
#define UI_TASK_STACK        8192     // UI任务堆栈大小
#define COMMAND_TASK_STACK   4096     // 命令处理任务堆栈大小
#define DEFAULT_PRIORITY     1        // 默认任务优先级
#define SENSOR_PRIORITY      2        // 传感器任务优先级
#define SENSOR_PERIOD_MS     1000     // 传感器采样周期(毫秒)
#define UI_PERIOD_MS         50       // UI更新周期(毫秒)
#define MAX_TEMP             150.0f   // 最高允许温度(摄氏度)
#define MIN_TEMP             0.0f     // 最低允许温度(摄氏度)
#define DEFAULT_TEMP         100.0f   // 默认目标温度
#define I2C_RETRY_COUNT      3        // I2C通信重试次数
#define PWM_FREQUENCY        5000     // PWM频率(Hz)
#define PWM_RESOLUTION       8        // PWM分辨率(bit)
#define PWM_CHANNEL          0        // PWM通道

// 自动调谐参数
#define AUTOTUNE_TEMP_DELTA  5.0f     // 自动调谐温度增量
#define AUTOTUNE_CYCLES      8        // 需要记录的峰谷数
#define AUTOTUNE_LOOKBACK    5        // 峰谷检测回溯点数

// 存储参数
#define PREF_NAMESPACE       "pid_cfg" // Preferences命名空间
#define PREF_KEY_KP          "kp"     // Kp存储键
#define PREF_KEY_KI          "ki"     // Ki存储键
#define PREF_KEY_KD          "kd"     // Kd存储键
#define PREF_KEY_TARGET      "target" // 目标温度存储键

// 调试配置
#define DEBUG_SERIAL         Serial   // 调试串口
#define SERIAL_BAUD_RATE     115200   // 串口波特率

// OLED显示配置 (可选)
#define OLED_ADDRESS         0x3C     // OLED I2C地址
#define OLED_WIDTH           128      // OLED宽度(像素)
#define OLED_HEIGHT          64       // OLED高度(像素)

#endif // CONFIG_HPP 