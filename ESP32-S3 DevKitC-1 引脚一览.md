# ESP32-S3 DevKitC-1 引脚一览

以下内容基于 Renzo Mischianti 的高分辨率引脚图及 Random Nerd Tutorials 教程整理。

---

## 电源与地
- **3.3V**：电源输入（左侧上方两脚）
- **5V**：USB 5V（左侧下方第二脚）
- **GND**：地（左侧最下脚 & 右侧最上及最下两脚）

---

## 模拟输入 (ADC)、电容触摸 & RTC

| GPIO  | ADC 通道     | 触摸通道 | RTC GPIO    |
|-------|--------------|----------|-------------|
| 1     | ADC1_CH0     | T1       | RTC_GPIO1   |
| 2     | ADC1_CH1     | T2       | RTC_GPIO2   |
| 3     | ADC1_CH2     | T3       | RTC_GPIO3   |
| 4     | ADC1_CH3     | T4       | RTC_GPIO4   |
| 5     | ADC1_CH4     | T5       | RTC_GPIO5   |
| 6     | ADC1_CH5     | T6       | RTC_GPIO6   |
| 7     | ADC1_CH6     | T7       | RTC_GPIO7   |
| 8     | ADC1_CH7     | T8       | RTC_GPIO8   |
| 9     | ADC1_CH8     | T9       | RTC_GPIO9   |
| 10    | ADC1_CH9     | T10      | RTC_GPIO10  |
| 11    | ADC2_CH0     | T11      | RTC_GPIO11  |
| 12    | ADC2_CH1     | T12      | RTC_GPIO12  |
| 13    | ADC2_CH2     | T13      | RTC_GPIO13  |
| 14    | ADC2_CH3     | T14      | RTC_GPIO14  |
| 15    | ADC2_CH4     | –        | RTC_GPIO15  |
| 16    | ADC2_CH5     | –        | RTC_GPIO16  |
| 17    | ADC2_CH6     | –        | RTC_GPIO17  |
| 18    | ADC2_CH7     | –        | RTC_GPIO18  |
| 19    | ADC2_CH8     | –        | RTC_GPIO19  |
| 20    | ADC2_CH9     | –        | RTC_GPIO20  |

以上 ADC、触摸、RTC 功能映射表格基于 Renzo Mischianti 引脚图及 Random Nerd Tutorials 数据整理。 [oai_citation:0‡Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/)

---

## I²C（默认）
- **SDA**：GPIO8  
- **SCL**：GPIO9  
（板载 I²C 默认口） [oai_citation:1‡Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/)

---

## SPI
- **HSPI（SPI2）**  
  - MOSI：GPIO11  
  - MISO：GPIO13  
  - SCLK：GPIO12  
  - SS：  GPIO10  
- **VSPI（SPI3）**  
  - MOSI：GPIO35  
  - MISO：GPIO37  
  - SCLK：GPIO36  
  - SS：  GPIO39  

> 注意：GPIO26–GPIO32 用于板载 Flash/PSRAM，建议勿复用。 [oai_citation:2‡Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/)

---

## UART
| UART  | TX GPIO | RX GPIO | 说明             |
|-------|---------|---------|------------------|
| UART0 | GPIO43  | GPIO44  | 板载 USB-Serial  |
| UART1 | GPIO17  | GPIO18  | 可重新映射       |
| UART2 | 任意    | 任意    | 用户自定义       |

默认 UART 引脚映射参考 Random Nerd Tutorials。 [oai_citation:3‡Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/)

---

## PWM
所有可输出 GPIO 均可配置为 PWM（8 通道 LED PWM 控制器）。

---

## Strapping 引脚
- **GPIO0**, **GPIO3**, **GPIO45**, **GPIO46**  
  用于启动模式选择，尽量避免外部电路干扰。 [oai_citation:4‡Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/)

---

## USB OTG
- **D+**：GPIO20  
- **D-**：GPIO19  

---

## 板载 LED
- **单色 LED**：GPIO38  
- **RGB LED**：GPIO48  

---