/**
 * @file autotune.hpp
 * @brief PID自动调谐模块，实现Ziegler-Nichols方法
 * @author Claude Assistant
 * @date 2023-11-15
 */

#ifndef _AUTOTUNE_HPP_
#define _AUTOTUNE_HPP_

#include <Arduino.h>
#include "config.hpp"
#include "pid.hpp"

/**
 * @brief 自动调谐状态
 */
enum class AutotuneState {
    IDLE,             ///< 空闲状态
    INIT,             ///< 初始化状态
    HEATING,          ///< 加热状态
    COOLING,          ///< 冷却状态
    COMPLETED,        ///< 完成状态
    FAILED            ///< 失败状态
};

/**
 * @brief 自动调谐数据点
 */
struct AutotunePoint {
    float temperature;    ///< 温度值
    uint32_t timestamp;   ///< 时间戳(毫秒)
    bool isPeak;          ///< 是否为峰值点
};

/**
 * @brief PID自动调谐类
 * 
 * 使用继电器反馈方法(Bang-Bang控制)来测量过程动态响应，
 * 然后应用Ziegler-Nichols规则计算PID参数。
 */
class PIDAutotuner {
public:
    /**
     * @brief 构造函数
     * @param targetTemp 目标温度
     * @param outputStep 输出步长(0.0-1.0)
     * @param tempDelta 温度波动范围
     * @param lookbackCount 峰值检测的回溯点数
     * @param maxCycles 最大循环次数
     */
    PIDAutotuner(
        float targetTemp, 
        float outputStep = 1.0f,
        float tempDelta = AUTOTUNE_TEMP_DELTA,
        uint8_t lookbackCount = AUTOTUNE_LOOKBACK,
        uint8_t maxCycles = AUTOTUNE_CYCLES
    );
    
    /**
     * @brief 开始自动调谐过程
     */
    void start();
    
    /**
     * @brief 重置自动调谐器
     */
    void reset();
    
    /**
     * @brief 更新自动调谐器
     * @param currentTemp 当前温度
     * @param dt 时间间隔(秒)
     * @return 加热器输出值(0.0-1.0)
     */
    float update(float currentTemp, float dt);
    
    /**
     * @brief 计算最终的PID参数
     * @param pidController 要更新的PID控制器
     * @return 是否成功计算出参数
     */
    bool computePIDParameters(PIDController* pidController);
    
    /**
     * @brief 获取当前状态
     * @return 自动调谐状态
     */
    AutotuneState getState() const;
    
    /**
     * @brief 检查自动调谐是否完成
     * @return 是否完成
     */
    bool isCompleted() const;
    
    /**
     * @brief 获取进度百分比
     * @return 进度(0-100)
     */
    uint8_t getProgress() const;
    
    /**
     * @brief 获取状态描述字符串
     * @param buffer 输出缓冲区
     * @param bufferSize 缓冲区大小
     */
    void getStatusString(char* buffer, size_t bufferSize) const;

private:
    /**
     * @brief 检测峰值和谷值
     * @param newTemp 新温度点
     * @return 是否检测到新的峰值或谷值
     */
    bool detectPeaks(float newTemp);
    
    float targetTemp;              ///< 目标温度
    float outputStep;              ///< 输出步长
    float tempDelta;               ///< 温度波动范围
    uint8_t lookbackCount;         ///< 峰值检测回溯点数
    uint8_t maxCycles;             ///< 最大周期数
    
    AutotuneState state;           ///< 当前状态
    
    // 温度历史记录
    float tempHistory[AUTOTUNE_LOOKBACK];
    uint8_t historyIndex;
    
    // 峰值和谷值记录
    AutotunePoint peaks[AUTOTUNE_CYCLES+1];
    uint8_t peakCount;
    
    // 计算结果
    float ku;                      ///< 临界增益
    float tu;                      ///< 临界周期(秒)
    float peakToTrough;            ///< 峰谷温差
    
    uint32_t startTime;            ///< 开始时间戳
    float lastOutput;              ///< 上次输出值
};

#endif // _AUTOTUNE_HPP_ 