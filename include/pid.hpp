/**
 * @file pid.hpp
 * @brief PID控制器模块
 * @author Claude Assistant
 * @date 2023-11-15
 */

#ifndef PID_HPP
#define PID_HPP

#include <Arduino.h>
#include "config.hpp"

/**
 * @brief PID控制器状态结构体
 */
typedef struct {
    float Kp;                // 比例系数
    float Ki;                // 积分系数
    float Kd;                // 微分系数
    float minDerivTime;      // 最小微分时间
    float integral;          // 积分累积值
    float prevTemp;          // 前一次温度读数
    float prevTempDerivative;// 前一次温度微分值
    float smoothFactor;      // 微分平滑因子
    float heaterMaxPower;    // 加热器最大功率(0.0-1.0)
    float integratorLimit;   // 积分限幅值
} PIDState;

/**
 * @class PIDController
 * @brief PID控制器类
 */
class PIDController {
private:
    PIDState state;
    
public:
    /**
     * @brief 构造函数
     * @param Kp 比例系数
     * @param Ki 积分系数
     * @param Kd 微分系数
     * @param minDerivTime 最小微分时间
     * @param maxPower 加热器最大功率(0.0-1.0)
     */
    PIDController(
        float Kp = PID_KP, 
        float Ki = PID_KI, 
        float Kd = PID_KD, 
        float minDerivTime = PID_MIN_DERIV_TIME, 
        float maxPower = HEATER_MAX_POWER
    );
    
    /**
     * @brief 重置PID控制器状态
     */
    void reset();
    
    /**
     * @brief 更新PID控制并计算输出
     * @param currentTemp 当前温度
     * @param targetTemp 目标温度
     * @param dt 自上次更新以来的时间间隔(秒)
     * @return 加热器输出功率(0.0-1.0)
     */
    float update(float currentTemp, float targetTemp, float dt);
    
    /**
     * @brief 设置PID参数
     * @param Kp 比例系数
     * @param Ki 积分系数
     * @param Kd 微分系数
     */
    void setParameters(float Kp, float Ki, float Kd);
    
    /**
     * @brief 设置最大输出功率
     * @param maxPower 最大功率(0.0-1.0)
     */
    void setMaxPower(float maxPower);
    
    /**
     * @brief 获取PID参数字符串表示
     * @param buffer 输出缓冲区
     * @param bufferSize 缓冲区大小
     */
    void getParametersString(char *buffer, size_t bufferSize) const;
    
    /**
     * @brief 获取PID状态引用
     * @return PID状态结构体引用
     */
    const PIDState& getState() const;
    
    /**
     * @brief 获取比例系数
     * @return 比例系数Kp
     */
    float getKp() const;
    
    /**
     * @brief 获取积分系数
     * @return 积分系数Ki
     */
    float getKi() const;
    
    /**
     * @brief 获取微分系数
     * @return 微分系数Kd
     */
    float getKd() const;
};

#endif // PID_HPP 