/**
 * @file encoder.hpp
 * @brief 旋转编码器模块，提供用户界面输入功能
 * @author Claude Assistant
 * @date 2023-11-15
 */

#ifndef _ENCODER_HPP_
#define _ENCODER_HPP_

#include <Arduino.h>
#include "config.hpp"

/**
 * @brief 编码器事件类型
 */
enum class EncoderEvent {
    NONE,        ///< 无事件
    CLOCKWISE,   ///< 顺时针旋转
    COUNTER_CW,  ///< 逆时针旋转
    CLICK,       ///< 按下按钮
    LONG_PRESS   ///< 长按按钮
};

/**
 * @brief 编码器状态结构体
 */
struct EncoderState {
    volatile int32_t position;     ///< 当前位置计数
    volatile bool buttonPressed;   ///< 按钮是否按下
    uint32_t buttonPressTime;      ///< 按钮按下时间戳(ms)
    bool longPressDetected;        ///< 是否检测到长按
};

/**
 * @brief 旋转编码器模块类
 */
class RotaryEncoder {
public:
    /**
     * @brief 构造函数
     * @param pinA A相引脚
     * @param pinB B相引脚
     * @param pinBtn 按钮引脚
     * @param longPressTime 长按判定时间(ms)
     */
    RotaryEncoder(uint8_t pinA, uint8_t pinB, uint8_t pinBtn, uint32_t longPressTime = 1000);
    
    /**
     * @brief 初始化编码器
     */
    void begin();
    
    /**
     * @brief 更新编码器状态
     * @return 检测到的事件类型
     */
    EncoderEvent update();
    
    /**
     * @brief 获取编码器计数位置
     * @return 编码器计数
     */
    int32_t getPosition() const;
    
    /**
     * @brief 重置编码器计数
     * @param newPosition 新的计数值
     */
    void setPosition(int32_t newPosition);
    
    /**
     * @brief 获取按钮状态
     * @return 按钮是否被按下
     */
    bool isButtonPressed() const;

private:
    /**
     * @brief A相引脚中断处理函数
     */
    static void IRAM_ATTR handleInterruptA();
    
    /**
     * @brief B相引脚中断处理函数
     */
    static void IRAM_ATTR handleInterruptB();
    
    /**
     * @brief 按钮引脚中断处理函数
     */
    static void IRAM_ATTR handleInterruptButton();
    
    uint8_t pinA;            ///< A相引脚
    uint8_t pinB;            ///< B相引脚
    uint8_t pinBtn;          ///< 按钮引脚
    uint32_t longPressTime;  ///< 长按时间(毫秒)
    
    static RotaryEncoder *instance;  ///< 静态实例指针(用于中断处理)
    EncoderState state;              ///< 编码器状态
};

#endif // _ENCODER_HPP_ 