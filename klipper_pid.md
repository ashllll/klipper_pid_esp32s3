# ESP32-S3 PID 温控项目总结

## 一、项目概述
基于 Klipper 固件中的 PID 控制逻辑，将闭环温度控制算法移植到 ESP32-S3 平台，结合 FreeRTOS，实现对 PTC 加热器的精准温度控制与自动调谐功能。

---

## 二、系统架构

### 1. 硬件组成
- **MCU**：ESP32-S3  
- **温度传感**：  
  - NTC 100 KΩ 热敏电阻 + 100 KΩ 上拉电阻  
  - ADS1115 16-bit ADC（3.3V供电）  
- **加热器**：12 V 100 W PTC，加 N 沟道 MOSFET 驱动  
- **通信接口**：I²C（MCU ↔ ADS1115）、串口/USB（调试与命令）  

### 2. 软件环境
- **开发平台**：PlatformIO + Arduino + FreeRTOS  
- **依赖库**：  
  - adafruit/Adafruit ADS1X15@^2.5.0（ADS1115 驱动）  
  - FreeRTOS （任务调度、信号量）  

---

## 三、关键模块说明

### 1. `include
- 硬件引脚定义：I²C SDA/SCL、PWM 输出引脚  
- 传感器参数：NTC Beta 值、`R0`、`T0`  
- 初始 PID 参数：`Kp`、`Ki`、`Kd`、`min_deriv_time`、`heater_max_power`  

### 2. `ads1115` 驱动（`.hpp` / `.cpp`）
- 初始化 I²C 与 ADS1115  
- 连续转换模式读取 A0 通道原始值  

### 3. 温度转换（`ntc.cpp` / `ntc.hpp`）
1. ADC 原始值 → 电压  
2. 分压计算 NTC 阻值  
3. Beta 公式：  
   $$
   \frac{1}{T} = \frac{1}{T_0} + \frac{1}{B} \ln\!\bigl(\tfrac{R_\text{NTC}}{R_0}\bigr)
   $$  
4. 转换为摄氏度  

### 4. PID 控制
- **状态结构** `PIDState`：存储 `Kp`、`Ki`、`Kd`、积分限幅、微分平滑常数、历史温度/微分  
- **初始化** `pidInit()`：设置各项参数及积分最大值  
- **更新** `pidUpdate(current, setpoint, dt)`：  
  1. 计算误差 `error = setpoint – current`  
  2. 更新并限幅积分（Anti-windup）  
  3. 平滑微分或直接微分  
  4. 按公式计算输出，并限幅至 [0,1]  
  5. 根据输出饱和状态决定是否继续积分  
  6. 返回占空比  

### 5. 自动调谐（`autotune.cpp` / `autotune.hpp`）
- **Bang–Bang 控制**：在目标温度 ±δ 切换  
- 记录峰/谷温度与时间  
- 计算临界增益 $Ku$ 与周期 $Tu$：  
  $$
  Ku = \frac{4}{\pi A},\quad Tu = \text{平均振荡周期}
  $$  
- Ziegler–Nichols 公式：  
Kp = 0.6·Ku
Ki = Kp / (0.5·Tu)
Kd = Kp · (0.125·Tu)

---

## 四、FreeRTOS 任务设计

**核心思想：** 使用队列进行任务间数据传递，确保数据同步与解耦。使用 `vTaskDelayUntil` 实现精确的周期性任务。

### 1. `temperatureQueue` (FreeRTOS Queue)
- 定义一个队列用于存储 `float` 类型的温度数据。
- `configQUEUE_REGISTRY_SIZE` 可能需要配置以在调试时命名队列。

### 2. SensorTask (传感器读取任务)
- **优先级：** 建议设置为中等或稍高优先级 (e.g., `tskIDLE_PRIORITY + 2`)。
- **周期：** 使用 `vTaskDelayUntil` 实现精确的 100 ms 周期。
- **流程：**
  1. 初始化 `TickType_t xLastWakeTime = xTaskGetTickCount();`
  2. **循环:**
     a. 调用 `ads.readRaw()` 获取原始 ADC 值。
     b. 调用 `rawToTemperature()` 将原始值转换为摄氏度 (`float currentTemp`)。
     c. 将 `currentTemp` 发送到 `temperatureQueue`：`xQueueSend(temperatureQueue, &currentTemp, portMAX_DELAY);` (或适当的超时)。检查返回值处理错误。
     d. 使用 `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));` 等待下一个周期。

### 3. ControlTask (控制逻辑任务)
- **优先级：** 建议设置为中等优先级 (e.g., `tskIDLE_PRIORITY + 1`)，可根据实时性要求调整。
- **流程：**
  1. **循环:**
     a. 阻塞等待从 `temperatureQueue` 接收温度数据：`xQueueReceive(temperatureQueue, &receivedTemp, portMAX_DELAY);`。检查返回值处理错误。
     b. 获取当前时间，计算自上次执行以来的时间差 `dt` (用于 PID 微分和积分项)。
     c. **调谐模式判断:**
        - **是调谐模式:**
          - 调用 `autotuneStep(receivedTemp)`。
          - 如果 `autotuneIsFinished()` 返回 true，则调用 `autotuneComputePID()` 计算新的 PID 参数并更新 `pidState`，然后切换回正常模式。
        - **否 (正常模式):**
          - 调用 `pidUpdate(pidState, receivedTemp, targetTemp, dt)` 获取 PID 输出值 (`output`)。
          - 将 `output` (范围 [0, 1]) 转换为 PWM 占空比 (e.g., 0-255 for 8-bit PWM)。
          - 调用 `pwmWrite(outputPwm)` 更新加热器 PWM 输出。
     d. 更新上次执行时间。

---

## 五、错误处理与硬件保护

- **I²C 重试**：读写 ADS1115 最多重试 3 次，失败后断电保护  
- **过温/传感故障**：检测温度范围（0 ℃–150 ℃）或恒定原始值，触发安全停机  
- **信号量与队列**：确保数据一致性，避免竞态  

---

## 六、接口与扩展

- **动态修改目标温度**：串口命令解析（如 `SET:100`），或通过 BLE/Web 接口接收  
- **切换调谐模式**：串口发送 `ATUNE` 命令  
- **持久化 PID 参数**：EEPROM/Flash 或串口命令读取并保存  
- **日志输出**：  
```cpp
Serial.printf("E=%.2f I=%.2f D=%.2f OUT=%.2f\n",
              error, pid.integral, temp_deriv, output);

