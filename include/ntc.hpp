/**
 * @file ntc.hpp
 * @brief NTC热敏电阻温度转换模块
 * @author Claude Assistant
 * @date 2023-11-15
 */

#ifndef NTC_HPP
#define NTC_HPP

#include <Arduino.h>
#include "config.hpp"
// #include <Adafruit_ADS1X15.h> // Comment out old include
#include <Adafruit_ADS1X15.h> // Restore Adafruit lib include
// #include <Wire.h>             // Remove Wire.h if not needed by other headers

/**
 * @class NTCSensor
 * @brief NTC温度传感器类
 */
class NTCSensor {
private:
    // Adafruit_ADS1115* adc; // Comment out old type
    Adafruit_ADS1115* adc; // Restore Adafruit type
    float beta;
    float r0;
    float t0;
    float seriesR;
    float refVoltage;
    uint8_t adcChannel;
    
    /**
     * @brief 计算NTC电阻值
     * @param voltage ADC读取的电压值
     * @return NTC的电阻值(欧姆)
     */
    float voltageToResistance(float voltage) const;
    
    /**
     * @brief 将NTC电阻值转换为温度
     * @param resistance NTC电阻值(欧姆)
     * @return 温度(摄氏度)
     */
    float resistanceToTemperature(float resistance) const;
    
public:
    /**
     * @brief 构造函数
     * @param adc ADS1115驱动指针
     * @param channel ADC通道编号(0-3)
     * @param beta NTC Beta值
     * @param r0 标称电阻值(欧姆)
     * @param t0 标称温度(摄氏度)
     * @param seriesR 串联电阻值(欧姆)
     * @param refVoltage 参考电压值(V)
     */
    NTCSensor(
        // Adafruit_ADS1115* adc, // Comment out old type
        Adafruit_ADS1115* adc, // Restore Adafruit type
        uint8_t channel = 0,
        float beta = NTC_BETA,
        float r0 = NTC_R0,
        float t0 = NTC_T0,
        float seriesR = NTC_SERIES_R,
        float refVoltage = REFERENCE_VOLTAGE
    );
    
    /**
     * @brief 读取温度
     * @return 温度(摄氏度)，错误时返回-273.15
     */
    float readTemperature();
    
    /**
     * @brief 检查温度值是否在合理范围内
     * @param temperature 要检查的温度值
     * @return 如果在MIN_TEMP和MAX_TEMP之间返回true，否则返回false
     */
    static bool isTemperatureValid(float temperature);
};

#endif // NTC_HPP 