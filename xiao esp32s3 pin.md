# Seeed Studio XIAO ESP32-S3 引脚定义

| 引脚功能 | GPIO   | 其他功能      |
| -------- | ------ | ------------ |
| TOUCH1   | GPIO1  | A0, D0       |
| TOUCH2   | GPIO2  | A1, D1       |
| TOUCH3   | GPIO3  | A2, D2       |
| TOUCH4   | GPIO4  | A3, D3       |
| TOUCH5   | GPIO5  | SDA, A4, D4  |
| TOUCH6   | GPIO6  | SCL, A5, D5  |
| TOUCH7   | GPIO7  | SCK, D7      |
| TOUCH8   | GPIO8  | MISO, D8     |
| TOUCH9   | GPIO9  | MOSI, D10    |
| –        | GPIO10 | A10, D10     |
| –        | GPIO43 | TX           |
| –        | GPIO44 | RX           |

## 电源引脚
- **5V**
- **3V3**
- **GND**

## 功能分类

- **触摸输入 (Touch)**：GPIO1–GPIO9
- **数字输入输出 (Digital)**：D0–D10
- **模拟输入 (Analog)**：A0–A5, A10
- **GPIO 通用输入输出 (General Purpose IO)**：GPIO43, GPIO44
- **I²C 总线**：SDA(GPIO5), SCL(GPIO6)
- **SPI 总线**：MOSI(GPIO9), MISO(GPIO8), SCK(GPIO7)
- **UART 通信接口**：TX(GPIO43), RX(GPIO44)

---

# 模块分配

## Rotary Encoder 模块

| 属性                     | 值                                                                   |
| ------------------------ | -------------------------------------------------------------------- |
| **模块型号**             | EC11                                                                 |
| **接口类型**             | GPIO                                                                 |
| **每转脉冲数**           | 20                                                                   |
| **工作电压**             | 3.3V                                                                 |
| **GPIO 引脚分配**        | • A 相 (pin_a)：GPIO1  
• B 相 (pin_b)：GPIO2  
• 按钮 (button)：GPIO3  |
| **消抖处理**             | 是                                                                   |
| **备注**                 | A 相先于 B 相为顺时针。按钮下拉至低电平。使用 GyverEncoder.h 库。 |

> **说明**：以上 GPIO 均在 XIAO-ESP32-S3 板上可用。

## ADS 转换器 模块

| 属性                   | 值                                              |
| ---------------------- | ----------------------------------------------- |
| **芯片**               | ADS1115 / ADS1015                               |
| **接口类型**           | I²C                                             |
| **通道数**             | 4                                               |
| **分辨率**             | 16-bit                                          |
| **放大器**             | 可编程增益放大器 (PGA)                          |
| **默认 I²C 地址**      | 0x48                                            |
| **I²C 引脚**           | SDA(GPIO5), SCL(GPIO6)                          |
| **NTC 传感器类型**     | NTC100K（串联 100K，2-wire 连接）               |
| **备注**               | 使用 Adafruit_ADS1X15 库读取电压，通过 Steinhart–Hart 算法换算温度。 |

> **已校准**：I²C 总线使用 XIAO-ESP32-S3 板上 SDA/SCL(GPIO5/6)。

## 其他模块 & 依赖

- **显示模块 (OLED)**  
  - 驱动芯片：SSD1306  
  - 分辨率：128×64  
  - 尺寸：0.96"/1.3"  
  - I²C 地址：0x3C

- **传感器**  
  - SHT40（I²C 地址 0x44，SDA=GPIO5，SCL=GPIO6）  
  - ADS1115（同上）

- **存储**  
  - Preferences（EEPROM 模拟）

- **网络**  
  - WiFi（内置）  
  - BLE（预留）

- **依赖库**  
  - U8g2  
  - Adafruit SHT4x  
  - Adafruit ADS1X15  
  - WiFi  
  - GyverEncoder

