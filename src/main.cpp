#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>            // For SSD1306 display
#include <Adafruit_ADS1X15.h>   // For ADS1115 ADC
#include <Adafruit_SHT4x.h>     // For SHT40 Sensor (New)
#include <GyverEncoder.h>       // For Rotary Encoder input (New)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <PID_v1.h>             // PID Control Library
#include <PID_AutoTune_v0.h>    // <<< Correct header name based on example
#include <Preferences.h>        // For Non-Volatile Storage

// --- NVS Namespace --- 
Preferences preferences; // Use default namespace "storage"
const char* nvs_key_kp = "pid_kp";
const char* nvs_key_ki = "pid_ki";
const char* nvs_key_kd = "pid_kd";

// --- Pin Definitions (Based on ESP32-S3 DevKitC-1 Standard) ---
// I2C Pins (for SSD1306 & ADS1115)
#define I2C_SDA_PIN  8  // DevKitC-1 Default SDA (GPIO8)
#define I2C_SCL_PIN  9  // DevKitC-1 Default SCL (GPIO9)

// PWM Output Pin (for Heater MOSFET)
#define PWM_HEATER_PIN 10 // DevKitC-1 GPIO10 (Any GPIO can be PWM) - !!! 请根据实际接线确认 !!!

// Rotary Encoder Pins (EC11)
#define ENC_A_PIN    1  // DevKitC-1 GPIO1
#define ENC_B_PIN    2  // DevKitC-1 GPIO2
#define ENC_SW_PIN   42  // DevKitC-1 GPIO3 (Strapping Pin - Use with caution) - 假设按钮下拉低电平有效

// --- Constants ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // SSD1306 I2C Address (或 0x3D) - 使用 I2C scanner 确认

// --- Global Objects ---
// U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ OLED_RESET);

// ADS1115 ADC
Adafruit_ADS1115 ads;

// SHT40 Sensor (New)
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// Rotary Encoder (GyverEncoder)
Encoder encoder(ENC_A_PIN, ENC_B_PIN, ENC_SW_PIN, TYPE1); // Assuming INPUT_PULLUP for switch (HIGH level idle)

// --- FreeRTOS Handles ---
SemaphoreHandle_t i2cMutex = NULL;             // Mutex for I2C bus access (U8G2 needs protection)
SemaphoreHandle_t setpointMutex = NULL;        // Mutex for target setpoint variable
QueueHandle_t temperatureQueue = NULL;       // Queue for SensorTask -> ControlTask (float)
QueueHandle_t pwmOutputQueue = NULL;         // Queue for ControlTask -> PWMOutputTask (uint8_t)
QueueHandle_t uiEventQueue = NULL;           // Queue for EncoderInputTask -> UITask (UIEvent enum)

TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t pwmOutputTaskHandle = NULL;
TaskHandle_t encoderInputTaskHandle = NULL;
TaskHandle_t uiTaskHandle = NULL;

// --- Global Variables ---
double g_targetSetpoint = 34.0; // Target temperature (protected by setpointMutex)
float g_currentTemp = -999.0;   // Current temperature (for display)
uint8_t g_currentPWM = 0;       // Current PWM (for display)
float g_sht40Temp = -999.0;     // SHT40 Temperature (for display, New)
float g_sht40Humidity = -999.0; // SHT40 Humidity (for display, New)
float g_ntcVoltage = -999.0;    // NTC Voltage (for display, New)

// Flag to indicate if PID values were loaded from NVS
bool g_pidLoadedFromNVS = false;

// --- Control Task State Enum (Shared with UI Task) ---
// Moved here to be globally accessible
typedef enum { STATE_INITIAL_HEATING, STATE_WAITING_FOR_STABILITY, STATE_TUNING, STATE_RUNNING } ControlState;
volatile ControlState g_controlState = STATE_INITIAL_HEATING; // Global variable for control state (updated by ControlTask)

// We might need a mutex for g_currentTemp/g_currentPWM if access becomes complex,
// but for now, ControlTask writes pwmOutputQueue, UITask reads temperatureQueue/pwmOutputQueue peeking.
// Let's add mutex for temp/pwm display variables for safety.
SemaphoreHandle_t displayDataMutex = NULL;

// --- Sensor Filtering ---
const float EMA_ALPHA = 0.2; // Smoothing factor for EMA filter (smaller = stronger filtering, slower response)
float g_filteredTemp = -999.0; // Initialize with an unlikely value
bool g_firstReading = true;     // Flag for initial EMA reading

// --- Temperature Calibration ---
float g_temperatureOffset = 0.0; // Temperature offset in degrees C (Measured 92 when actual was 80)

// --- PID Variables (Defaults, might be overwritten by NVS) ---
// double Kp=2, Ki=5, Kd=1; // Initial PID parameters (will be tuned) - Original Defaults
double Kp=1.0, Ki=0.5, Kd=0.1; // Conservative starting parameters for manual tuning
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- AutoTune Variables ---
byte ATuneModeRemember=0;
// double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100; // AutoTune parameters - adjust these! (Original)
// double aTuneStep=25, aTuneNoise=1, aTuneStartValue=100; // Reduced aTuneStep (Still too high)
double aTuneStep=10, aTuneNoise=1, aTuneStartValue=100; // Drastically reduced aTuneStep to prevent overshoot during tuning
unsigned int aTuneLookBack=20; // AutoTune Lookback Seconds - adjust this!
PID_ATune aTune(&Input, &Output);    // <<< Correct class name based on example
boolean tuning = false; // Flag to indicate if tuning is running

// --- UI Event Enum ---
typedef enum { NAV_UP, NAV_DOWN, CONFIRM, BACK, LONG_PRESS, NO_EVENT } UIEvent;

// --- Task Function Declarations ---
void sensorTask(void *pvParameters);
void controlTask(void *pvParameters);
void pwmOutputTask(void *pvParameters);
void encoderInputTask(void *pvParameters);
void uiTask(void *pvParameters);

// --- Helper Function --- Convert ControlState enum to String ---
const char* controlStateToString(ControlState state) {
    switch (state) {
        case STATE_INITIAL_HEATING: return "加热中";
        case STATE_WAITING_FOR_STABILITY: return "等待稳定";
        case STATE_TUNING:          return "PID调谐";
        case STATE_RUNNING:         return "运行中";
        default:                    return "未知状态";
    }
}

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    unsigned long setupStartTime = millis();
    while (!Serial && millis() - setupStartTime < 2000) { // Shortened timeout
        delay(10);
    }
    Serial.println("\nESP32-S3 PID Controller Booting...");
    Serial.printf("Initial Heap: %u\n", ESP.getFreeHeap());

    // Initialize Mutexes
    Serial.println("Initializing Mutexes...");
    i2cMutex = xSemaphoreCreateMutex();
    setpointMutex = xSemaphoreCreateMutex();
    displayDataMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL || setpointMutex == NULL || displayDataMutex == NULL) {
        Serial.println("Error: Could not create Mutexes! Halting.");
        while(1);
    }
    Serial.printf("Heap after Mutexes: %u\n", ESP.getFreeHeap());

    // Initialize Queues
    Serial.println("Initializing Queues...");
    temperatureQueue = xQueueCreate(5, sizeof(float));
    pwmOutputQueue = xQueueCreate(5, sizeof(uint8_t));
    uiEventQueue = xQueueCreate(10, sizeof(UIEvent));
    if (!temperatureQueue || !pwmOutputQueue || !uiEventQueue ) {
         Serial.println("Error: Could not create Queues! Halting.");
         while(1);
    }
    Serial.printf("Heap after Queues: %u\n", ESP.getFreeHeap());

    // Initialize I2C Display (U8G2) - Protected by Mutex
    Serial.println("Initializing Display (U8G2)...");
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (!u8g2.begin()) {
            Serial.println("Error: Failed to initialize U8G2 display! Check wiring/address (0x3C?).");
        } else {
            Serial.println("Display Initialized.");
            // Initial message drawn via Astra UI init later
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("Timeout taking I2C Mutex for Display Init!");
    }
    Serial.printf("Heap after Display Init: %u\n", ESP.getFreeHeap());

    // Initialize ADS1115 - Protected by Mutex
    Serial.println("Initializing ADS1115...");
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        ads.setGain(GAIN_ONE);
        if (!ads.begin(0x48)) { // TODO: Make configurable if needed
            Serial.println("Error: Failed to initialize ADS1115! Check wiring/address (0x48?).");
        } else {
            Serial.println("ADS1115 Initialized.");
        }
        xSemaphoreGive(i2cMutex);
    } else {
         Serial.println("Timeout taking I2C Mutex for ADS1115 Init!");
    }
    Serial.printf("Heap after ADS1115 Init: %u\n", ESP.getFreeHeap());

    // Initialize SHT40 - Protected by Mutex (New)
    Serial.println("Initializing SHT40...");
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (!sht4.begin(&Wire)) { // Use the default I2C address 0x44
            Serial.println("Error: Failed to initialize SHT40! Check wiring/address (0x44?).");
        } else {
            Serial.println("SHT40 Initialized.");
            // Optional: Set precision and heater settings
            // sht4.setPrecision(SHT4X_HIGH_PRECISION);
            // sht4.setHeater(SHT4X_NO_HEATER);
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("Timeout taking I2C Mutex for SHT40 Init!");
    }
    Serial.printf("Heap after SHT40 Init: %u\n", ESP.getFreeHeap());

    // Initialize PWM Output (LEDC)
    Serial.println("Initializing PWM (LEDC)...");
    const int pwmChannel = 0;
    const double pwmFreq = 5000;
    const int pwmResolution = 8;
    ledcSetup(pwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(PWM_HEATER_PIN, pwmChannel);
    Serial.printf("PWM Initialized: Pin %d, Channel %d, Freq %.0f Hz, Res %d bit\n", PWM_HEATER_PIN, pwmChannel, pwmFreq, pwmResolution);
    Serial.printf("Heap after PWM Init: %u\n", ESP.getFreeHeap());

    // --- Initialize PID ---
    Serial.println("Initializing PID...");
    preferences.begin("pid-storage", false); // Start NVS, namespace "pid-storage", read/write
    bool nvs_has_pid = preferences.isKey(nvs_key_kp); // Check if Kp exists (assume others do too if Kp does)

    if (nvs_has_pid) {
        Kp = preferences.getDouble(nvs_key_kp, Kp); // Load Kp, use default if error
        Ki = preferences.getDouble(nvs_key_ki, Ki); // Load Ki
        Kd = preferences.getDouble(nvs_key_kd, Kd); // Load Kd
        Serial.printf("Loaded PID Tunings from NVS: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
        g_pidLoadedFromNVS = true; // Set flag indicating successful load
    } else {
        Serial.println("No PID Tunings found in NVS, using default values.");
        // Kp, Ki, Kd retain their default values defined above
    }
    preferences.end(); // Close NVS for now

    if(xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
        Setpoint = g_targetSetpoint;
        xSemaphoreGive(setpointMutex);
    } else {
        Serial.println("Setup: Failed to get initial setpoint!");
        Setpoint = 60.0; // Fallback
    }
    myPID.SetMode(AUTOMATIC); // Start in automatic mode for initial heating
    myPID.SetSampleTime(100); // Match task frequency?
    myPID.SetOutputLimits(0, 255);
    myPID.SetTunings(Kp, Ki, Kd); // Apply loaded/default tunings
    Serial.println("PID Initialized (Pre-tuning).");

    // --- Initialize AutoTune ---
    // Set AutoTune parameters - THESE MAY NEED ADJUSTMENT
    aTune.SetOutputStep(aTuneStep);
    aTune.SetControlType(1); // 1 = PID, 0 = PI
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetLookbackSec((int)aTuneLookBack);
    Serial.println("AutoTune Initialized.");

    // Create FreeRTOS Tasks
    Serial.println("Creating Tasks...");
    BaseType_t taskCreationResult;
    taskCreationResult = xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 3, &sensorTaskHandle);
    if (taskCreationResult != pdPASS) Serial.println("Error: Failed to create SensorTask!");
    taskCreationResult = xTaskCreate(controlTask, "ControlTask", 4096, NULL, 2, &controlTaskHandle); // Increase stack? Autotune might need it.
    if (taskCreationResult != pdPASS) Serial.println("Error: Failed to create ControlTask!");
    taskCreationResult = xTaskCreate(pwmOutputTask, "PWMOutputTask", 2048, NULL, 2, &pwmOutputTaskHandle);
    if (taskCreationResult != pdPASS) Serial.println("Error: Failed to create PWMOutputTask!");
    taskCreationResult = xTaskCreate(encoderInputTask, "EncoderInputTask", 2048, NULL, 4, &encoderInputTaskHandle);
    if (taskCreationResult != pdPASS) Serial.println("Error: Failed to create EncoderInputTask!");
    taskCreationResult = xTaskCreate(uiTask, "UITask", 4096, NULL, 1, &uiTaskHandle); // UI task might need more stack
    if (taskCreationResult != pdPASS) Serial.println("Error: Failed to create UITask!");

    Serial.printf("Heap after Task Creation: %u\n", ESP.getFreeHeap());
    if (ESP.getFreeHeap() < 8000) { // Increased warning threshold due to AutoTune potentially needing more RAM
        Serial.println("Warning: Low heap memory after task creation!");
    }
    Serial.println("Setup Complete. Starting Scheduler (implicitly).");
}

// --- Main Loop (Not used with FreeRTOS tasks, but required by Arduino framework) ---
void loop() {
  vTaskDelay(portMAX_DELAY); // Keep the loop task suspended
}

// --- Task Function Definitions ---

// Reads sensor, calculates temperature, sends to queue, updates global display var
void sensorTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Run at 10Hz
    float currentRawTemp = 0.0; // Renamed to indicate raw value before filtering
    int16_t adcValue = 0;
    float ntcVoltageLocal = 0.0; // Local variable for voltage calculation
    sensors_event_t humidity_event, temp_event; // For SHT40 reading

    const float NTC_BETA = 3950.0; // Example
    const float NTC_R0 = 100000.0;
    const float NTC_T0 = 25.0 + 273.15;
    const float R_SERIES = 100000.0;
    const float ADC_VREF = 3.3; // Use 3.3V as reference

    xLastWakeTime = xTaskGetTickCount();
    Serial.println("SensorTask started.");

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) { // Shorter timeout
            // 1. Read ADS1115 (NTC)
            adcValue = ads.readADC_SingleEnded(0); // Read from channel 0

            // 2. Read SHT40 (Temp/Humidity)
            bool sht40ReadSuccess = sht4.getEvent(&humidity_event, &temp_event); // Attempt to read SHT40

            xSemaphoreGive(i2cMutex); // Release mutex after I2C operations

             // --- Process ADS1115 (NTC) Data --- 
             // Use the ADC library's voltage conversion method if available and reliable,
             // otherwise calculate manually. ADS1115 has internal reference.
             // Let's use the provided PGA gain setting (GAIN_ONE = +/- 4.096V)
             ntcVoltageLocal = adcValue * (4.096 / 32767.0);

            // Clamp voltage to prevent issues, but also print if out of expected range
            if (ntcVoltageLocal < 0 || ntcVoltageLocal > ADC_VREF + 0.1) { // Allow slightly over VREF
                // Serial.printf("SensorTask: Warning! ntcVoltage out of range: %.3f V (ADC: %d)\n", ntcVoltageLocal, adcValue);
                if (ntcVoltageLocal < 0) ntcVoltageLocal = 0.001; // Avoid 0 for calculation
                if (ntcVoltageLocal > ADC_VREF) ntcVoltageLocal = ADC_VREF; // Clamp strictly for calculation
            } else if (ntcVoltageLocal < 0.01) {
                ntcVoltageLocal = 0.01; // Prevent division by zero or extremely large resistance values
            }

            // Update global NTC voltage for UI
            if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                 g_ntcVoltage = ntcVoltageLocal;
                 // We will update other globals later in this loop
                 xSemaphoreGive(displayDataMutex);
             } else {
                 // Serial.println("SensorTask: Failed mutex for g_ntcVoltage");
             }

            float ntcResistance = 0.0;
            // Check division boundaries
            if (ntcVoltageLocal < ADC_VREF - 0.01 && ntcVoltageLocal > 0.01) {
                // Correct formula for NTC pull-up: VREF -- NTC -- ADC -- R_SERIES -- GND
                ntcResistance = R_SERIES * (ADC_VREF - ntcVoltageLocal) / ntcVoltageLocal;
            } else if (ntcVoltageLocal <= 0.01) { // Voltage close to 0 (high resistance)
                 ntcResistance = 10000000.0; // Assign a very large resistance (e.g., 10 MOhm)
                 // Serial.printf("SensorTask: Warning! ntcVoltage (%.3f) close to 0, assuming high resistance.\n", ntcVoltageLocal);
            } else { // Voltage is very close to or above VREF (low resistance or error)
                 ntcResistance = 1.0; // Assign a small positive resistance (e.g., 1 Ohm)
                 // Serial.printf("SensorTask: Warning! ntcVoltage (%.3f) close to VREF (%.3f), assuming low resistance.\n", ntcVoltageLocal, ADC_VREF);
            }

            // Serial.printf("SensorTask: ADC=%d, V=%.3f, R=%.1f\n", adcValue, ntcVoltageLocal, ntcResistance); // Reduce verbosity


            if (ntcResistance > 0) {
                double log_arg = (double)ntcResistance / NTC_R0;
                if (log_arg > 0) {
                    // Using Beta formula
                    float steinhart = log(log_arg) / NTC_BETA + (1.0 / NTC_T0);
                    currentRawTemp = (1.0 / steinhart) - 273.15;

                     // Apply calibration offset before using the temperature value
                     currentRawTemp -= g_temperatureOffset;

                     // Sanity check temperature range AFTER offset (e.g., -20 to 300 C)
                     if (currentRawTemp < -20.0 || currentRawTemp > 300.0) {
                         // Serial.printf("SensorTask: Warning! Calculated Temp (after offset) out of range: %.1f C (R=%.1f)\n", currentRawTemp, ntcResistance);
                         // Decide how to handle: send error code or clamp? Sending error code for now.
                         currentRawTemp = -998.0; // Different error code
                     }

                } else {
                    // Serial.printf("SensorTask: Error! Invalid log argument: %.3f\n", log_arg);
                    currentRawTemp = -997.0; // Assign different error code
                }
            } else {
                // Serial.printf("SensorTask: Error! Calculated non-positive resistance: %.1f\n", ntcResistance);
                currentRawTemp = -999.0;
            }

            // Apply Exponential Moving Average (EMA) filter
            if (currentRawTemp > -900.0) { // Only filter valid temperature readings
                if (g_firstReading) {
                    g_filteredTemp = currentRawTemp; // Initialize filter with the first valid reading
                    g_firstReading = false;
                } else {
                    g_filteredTemp = EMA_ALPHA * currentRawTemp + (1.0 - EMA_ALPHA) * g_filteredTemp;
                }
            } else {
                 // If reading is invalid, pass it through without filtering (or maybe hold the last good value?)
                 // Passing it through for now so ControlTask sees the error codes
                 g_filteredTemp = currentRawTemp;
            }

            // Send temperature (filtered and offset) to control task queue
            if (xQueueSend(temperatureQueue, &g_filteredTemp, 0) != pdPASS) {
                 // Serial.println("Warning: Temperature queue full!");
            } else {
                 // Serial.printf("SensorTask Sent Temp: %.2f C\n", currentRawTemp);
            }

            // Update global variable (filtered and offset) for UI display (protected by mutex)
            if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_currentTemp = g_filteredTemp; // Update global with filtered value

                // --- Process SHT40 Data (Update Globals) ---
                if (sht40ReadSuccess) {
                    g_sht40Temp = temp_event.temperature;
                    g_sht40Humidity = humidity_event.relative_humidity;
                } else {
                    // Keep previous values or set error codes?
                    g_sht40Temp = -996.0; // Indicate SHT40 read error
                    g_sht40Humidity = -996.0;
                    // Serial.println("SensorTask: Failed to read SHT40!");
                }
                // g_ntcVoltage was updated earlier when its value was calculated

                xSemaphoreGive(displayDataMutex);
            }

        } else {
             Serial.println("Sensor Task: Failed to take I2C Mutex!");
        }
    }
}

// --- Helper Function for AutoTune Finish ---
void FinishAutoTune()
{
  Kp = aTune.GetKp();
  Ki = aTune.GetKi();
  Kd = aTune.GetKd();
  myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetMode(AUTOMATIC); // Turn PID back on
  tuning = false; // Clear tuning flag
  Serial.println("****************************************");
  Serial.println("PID AutoTune Complete!");
  Serial.printf ("New Tunings: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);

  // --- Save Tunings to NVS ---
  preferences.begin("pid-storage", false); // Open NVS again (read/write)
  preferences.putDouble(nvs_key_kp, Kp);
  preferences.putDouble(nvs_key_ki, Ki);
  preferences.putDouble(nvs_key_kd, Kd);
  preferences.end(); // Close NVS
  Serial.println("PID Tunings saved to NVS.");
  Serial.println("****************************************");
}

// Reads temperature, updates PID or AutoTune, sends PWM output
void controlTask(void *pvParameters) {
    float receivedTemp = 0.0;
    uint8_t pwmDuty = 0;
    unsigned long lastTimeNearSetpoint = 0;
    const unsigned long timeToStayNearSetpoint = 5000; // ms (5 seconds near setpoint before tuning)
    bool nearSetpointSustained = false;
    bool nearSetpoint = false; // Declare before switch
    bool isStable = false;     // Declare before switch

    // typedef enum { STATE_INITIAL_HEATING, STATE_WAITING_FOR_STABILITY, STATE_TUNING, STATE_RUNNING } ControlState; // Moved global

    // Set initial state based on whether PID values were loaded from NVS
    ControlState currentState; // Local state for this task
    if (g_pidLoadedFromNVS) {
        currentState = STATE_RUNNING; // PID loaded, start directly in running state
        Serial.println("ControlTask: PID loaded from NVS, starting in RUNNING state.");
        if(myPID.GetMode() == MANUAL) myPID.SetMode(AUTOMATIC); // Ensure PID is ON
    } else {
        currentState = STATE_INITIAL_HEATING; // No PID saved, start from initial heating and tuning sequence.
        Serial.println("ControlTask: No PID in NVS, starting initial heating and tuning sequence.");
    }
    // --- Initialize global state ---
    if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_controlState = currentState;
        xSemaphoreGive(displayDataMutex);
    } else {
        Serial.println("ControlTask: Failed init g_controlState mutex!");
    }


    Serial.println("ControlTask started.");
    unsigned long lastLogTime = 0;
    const unsigned long logInterval = 1000; // Log state every 1 second

    for (;;) {
        if (xQueueReceive(temperatureQueue, &receivedTemp, portMAX_DELAY) == pdPASS) {
            Input = (double)receivedTemp; // Update PID/AutoTune input

            // --- Get current Setpoint ---
            if(xSemaphoreTake(setpointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Only update PID's setpoint if the global one changed significantly
                if (abs(g_targetSetpoint - Setpoint) > 0.01) {
                     Setpoint = g_targetSetpoint;
                     Serial.printf("ControlTask: Setpoint updated to %.1f\n", Setpoint);
                }
                xSemaphoreGive(setpointMutex);
             }

             // --- Log Current State Periodically ---
             unsigned long now = millis();
             if (now - lastLogTime > logInterval) {
                 Serial.printf("ControlTask State: %d, Input: %.2f, Setpoint: %.1f, PWM: %d\n", (int)currentState, Input, Setpoint, pwmDuty);
                 lastLogTime = now;
             }

            // --- State Machine ---
            switch(currentState) {
                case STATE_INITIAL_HEATING:
                    // Add logging for the condition check
                    nearSetpoint = (Input >= Setpoint * 0.9 && Input > 0);
                    if (nearSetpoint) {
                        Serial.println("ControlTask: Reached near setpoint, waiting for stability...");
                        currentState = STATE_WAITING_FOR_STABILITY;
                        // --- Update global state ---
                        if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            g_controlState = currentState;
                            xSemaphoreGive(displayDataMutex);
                        }
                        lastTimeNearSetpoint = millis();
                        nearSetpointSustained = false;
                    }
                    // Run PID normally during heating
                    if(myPID.GetMode() == MANUAL) myPID.SetMode(AUTOMATIC); // Ensure PID is on
                    myPID.Compute();
                    pwmDuty = (uint8_t)Output;
                    break;

                case STATE_WAITING_FOR_STABILITY:
                    // Check if temperature stays near setpoint
                    isStable = (abs(Setpoint - Input) < 5.0);
                    if (isStable) { // Within 5 degrees C
                        if (millis() - lastTimeNearSetpoint > timeToStayNearSetpoint) {
                            nearSetpointSustained = true;
                        }
                    } else {
                        // Temperature moved away, reset timer
                        lastTimeNearSetpoint = millis();
                        nearSetpointSustained = false;
                    }

                    if (nearSetpointSustained) {
                        // Temperature is stable, proceed to tuning (will be skipped on next boot if saved)
                        Serial.println("ControlTask: Temperature stable near setpoint. Starting AutoTune.");
                        currentState = STATE_TUNING;
                         // --- Update global state ---
                        if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            g_controlState = currentState;
                            xSemaphoreGive(displayDataMutex);
                        }
                        tuning = true;
                        ATuneModeRemember = myPID.GetMode(); // Remember PID mode
                        // Configure AutoTune Start Value (based on current output?)
                        aTuneStartValue = Output; // Start tuning from current PID output
                        aTune.SetOutputStep(aTuneStep);
                        aTune.SetNoiseBand(aTuneNoise);
                        aTune.SetLookbackSec((int)aTuneLookBack);
                        myPID.SetMode(MANUAL); // Turn off PID controller
                    } else {
                        // Still waiting for stability, run PID normally
                         if(myPID.GetMode() == MANUAL) myPID.SetMode(AUTOMATIC);
                         myPID.Compute();
                         pwmDuty = (uint8_t)Output;
                    }
                    break;

                case STATE_TUNING:
                    if (tuning) {
                        byte val = aTune.Runtime();
                        if (val != 0) { // Tuning finished
                            FinishAutoTune();
                            currentState = STATE_RUNNING;
                             // --- Update global state ---
                            if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                g_controlState = currentState;
                                xSemaphoreGive(displayDataMutex);
                            }
                            // After tuning, use the PID output for the first cycle
                            myPID.Compute();
                            pwmDuty = (uint8_t)Output;
                        } else {
                            // Tuning still running, use AutoTune's output
                            pwmDuty = (uint8_t)Output; // Output is updated by aTune object
                        }
                    } else {
                         // Should not happen in this state, but fallback to running PID
                         Serial.println("ControlTask: Error! Tuning flag false in TUNING state. Switching to RUNNING.");
                         currentState = STATE_RUNNING;
                          // --- Update global state ---
                          if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            g_controlState = currentState;
                            xSemaphoreGive(displayDataMutex);
                          }
                         if(myPID.GetMode() == MANUAL) myPID.SetMode(AUTOMATIC);
                         myPID.Compute();
                         pwmDuty = (uint8_t)Output;
                    }
                    break;

                case STATE_RUNNING:
                     // Handle sensor errors
                     if (Input <= -900.0) {
                         if(myPID.GetMode() == AUTOMATIC) myPID.SetMode(MANUAL);
                         Output = 0;
                         pwmDuty = 0;
                     } else {
                         if(myPID.GetMode() == MANUAL) {
                             myPID.SetMode(AUTOMATIC); // Re-enable PID if it was off
                         }
                         myPID.Compute();
                         pwmDuty = (uint8_t)Output;
                     }
                    break;
            } // End State Machine


             // --- Update global PWM variable for UI ---
              if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                 g_currentPWM = pwmDuty;
                 // Also update g_controlState here in case it changed due to error handling in STATE_RUNNING etc.
                 // Although the primary updates are at state transitions, ensure consistency.
                 // Note: currentState is local, g_controlState is global. We check if the *global* needs update based on local.
                 // This might be slightly redundant if updates always happen at transitions, but safer.
                 if (g_controlState != currentState) {
                     g_controlState = currentState; // Update if local state differs (e.g., error recovery moved state)
                 }
                 xSemaphoreGive(displayDataMutex);
              }

            // --- Send PWM to output task ---
            // Serial.printf("ControlTask: Sending PWM=%d to Queue=0x%X\n", pwmDuty, (uintptr_t)pwmOutputQueue); // Keep debug for now
            if (xQueueSend(pwmOutputQueue, &pwmDuty, pdMS_TO_TICKS(10)) != pdPASS) {
                 // Serial.println("Warning: PWM Output Queue full!");
            }

        } // End if xQueueReceive
    } // End for(;;)
}


// Reads PWM value from queue, updates hardware PWM
void pwmOutputTask(void *pvParameters) {
    uint8_t pwmDuty = 0;
    const int pwmChannel = 0;
    Serial.println("PWMOutputTask started.");
    for (;;) {
        if (xQueueReceive(pwmOutputQueue, &pwmDuty, portMAX_DELAY) == pdPASS) {
             // Serial.printf("PWMOutputTask Received PWM: %d\n", pwmDuty); // Reduce verbosity
             ledcWrite(pwmChannel, pwmDuty);
        }
    }
}

// Reads encoder, sends UI events to queue
void encoderInputTask(void *pvParameters) {
    encoder.setType(TYPE1);
    encoder.setTickMode(AUTO);
    Serial.println("EncoderInputTask started.");
    for (;;) {
        encoder.tick();
        UIEvent eventToSend = NO_EVENT;

        if (encoder.isTurn()) {
            eventToSend = encoder.isRight() ? NAV_DOWN : NAV_UP;
        } else if (encoder.isClick()) {
            eventToSend = CONFIRM;
            // Serial.println(">>> Event: CONFIRM (Single Click)"); // Reduce verbosity
        } else if (encoder.isDouble()) {
             eventToSend = BACK;
             // Serial.println(">>> Event: BACK (Double Click)"); // Reduce verbosity
        } else if (encoder.isHolded()) {
             eventToSend = LONG_PRESS; // Optional: Define behavior for long press in UI
             // Serial.println(">>> Event: LONG_PRESS (Holded)"); // Reduce verbosity
        }

        if (eventToSend != NO_EVENT) {
            if (xQueueSend(uiEventQueue, &eventToSend, 0) != pdPASS) {
                // Serial.println("Warning: UI Event Queue full!");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay important even with AUTO mode
    }
}

// Handles UI logic and drawing using U8g2
void uiTask(void *pvParameters) {
    UIEvent receivedEvent = NO_EVENT;
    float tempToDisplay = -999.0;
    uint8_t pwmToDisplay = 0;
    double setpointToDisplay = 0.0;
    ControlState controlStateToDisplay = STATE_INITIAL_HEATING; // Local copy of control state

    typedef enum { STATE_HOME, STATE_SETTINGS_MENU, STATE_EDIT_SETPOINT } UIState;
    UIState currentState = STATE_HOME;
    int selectedMenuItem = 0;
    double editingSetpoint = 0.0;
    int homePageIndex = 0; // 0: Main Control, 1: Sensors & Params
    const int numHomePageIndexes = 2;
    const char *menuItems[] = {"Target Temp", "Start Tune", "Exit"}; // Added Start Tune option
    const int numMenuItems = sizeof(menuItems) / sizeof(menuItems[0]);
    bool redrawRequired = true;
    float prevTempToDisplay = -1000.0;
    uint8_t prevPwmToDisplay = 255;
    double prevSetpointToDisplay = -1000.0;
    ControlState prevControlStateToDisplay = (ControlState)-1; // Initialize to invalid state
    unsigned long lastRedrawTime = 0;
    const unsigned long redrawInterval = 100; // Redraw max 10 FPS to reduce load
    unsigned long blinkStartTime = 0;
    bool blinkState = false;
    const unsigned long blinkInterval = 300; // Blinking speed for menu cursor
    unsigned long lastNavEventTime = 0; // Timestamp for home screen navigation debounce
    const unsigned long navDebounceInterval = 100; // Debounce time in ms


    Serial.println("UITask started.");

    char statusBuffer[40]; // Increased size for tuning status
    char tempStr[10];

    for (;;) {
        // --- Handle Input Events ---
        if (xQueueReceive(uiEventQueue, &receivedEvent, 0) == pdPASS) {
            // Serial.printf("UI Task: Received Event %d\n", receivedEvent); // Reduce verbosity
            redrawRequired = true; // Any event requires redraw

            switch (currentState) {
                case STATE_HOME:
                    if (receivedEvent == NAV_UP || receivedEvent == NAV_DOWN) {
                        unsigned long currentMillis = millis();
                        if (currentMillis - lastNavEventTime > navDebounceInterval) {
                            if (receivedEvent == NAV_UP) {
                                homePageIndex = (homePageIndex - 1 + numHomePageIndexes) % numHomePageIndexes;
                            } else { // NAV_DOWN
                                homePageIndex = (homePageIndex + 1) % numHomePageIndexes;
                            }
                            lastNavEventTime = currentMillis; // Update timestamp only when event is processed
                        } else {
                            redrawRequired = false; // Ignore event, don't redraw
                        }
                    } else if (receivedEvent == CONFIRM) {
                        currentState = STATE_SETTINGS_MENU;
                        selectedMenuItem = 0;
                    }
                    break;

                case STATE_SETTINGS_MENU:
                    switch (receivedEvent) {
                        case NAV_UP:
                            selectedMenuItem = (selectedMenuItem - 1 + numMenuItems) % numMenuItems;
                            break;
                        case NAV_DOWN:
                            selectedMenuItem = (selectedMenuItem + 1) % numMenuItems;
                            break;
                        case CONFIRM:
                            if (selectedMenuItem == 0) { // Target Temp
                                currentState = STATE_EDIT_SETPOINT;
                                if(xSemaphoreTake(setpointMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                    editingSetpoint = g_targetSetpoint;
                                    xSemaphoreGive(setpointMutex);
                                } else { editingSetpoint = 60.0;} // Fallback
                            } else if (selectedMenuItem == 1) { // Start Tune
                                // TODO: How to signal controlTask to start tuning manually?
                                // For now, this option doesn't do anything if auto-tuning is automatic
                                Serial.println("UI: Manual Tune Start requested (Not fully implemented for auto-mode yet)");
                                // If we want manual trigger:
                                // 1. Add a flag `g_manualTuneRequested = true;`
                                // 2. `controlTask` checks this flag in RUNNING state to transition to TUNING.
                                currentState = STATE_HOME; // Go back home after request
                            } else if (selectedMenuItem == 2) { // Exit
                                currentState = STATE_HOME;
                            }
                            break;
                        case BACK:
                            currentState = STATE_HOME;
                            break;
                        default: break;
                    }
                    break;

                case STATE_EDIT_SETPOINT:
                    switch (receivedEvent) {
                        case NAV_UP:
                            editingSetpoint += 0.5;
                            break;
                        case NAV_DOWN:
                            editingSetpoint -= 0.5;
                            if (editingSetpoint < 0) editingSetpoint = 0;
                            break;
                        case CONFIRM:
                            if(xSemaphoreTake(setpointMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                g_targetSetpoint = editingSetpoint;
                                Serial.printf("UI: Setpoint updated to: %.1f\n", g_targetSetpoint);
                                xSemaphoreGive(setpointMutex);
                                // Signal PID to update its internal setpoint quickly
                                // This happens automatically in controlTask loop now
                            } else {
                                Serial.println("UI Task: Failed to take setpointMutex to confirm edit!");
                            }
                            currentState = STATE_SETTINGS_MENU;
                            break;
                        case BACK:
                            currentState = STATE_SETTINGS_MENU;
                            break;
                        default: break;
                    }
                    break;
            } // End State Machine Switch
            receivedEvent = NO_EVENT; // Consume event
        } // End Event Handling


        // --- Rate Limit Redraw ---
         if (millis() - lastRedrawTime < redrawInterval && !redrawRequired) {
             vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield
             continue;
         }

        // --- Get latest data for display ---
        if(xSemaphoreTake(displayDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            tempToDisplay = g_currentTemp;
            pwmToDisplay = g_currentPWM;
            controlStateToDisplay = g_controlState; // Get current control state
            xSemaphoreGive(displayDataMutex);
        }
        if(xSemaphoreTake(setpointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            setpointToDisplay = g_targetSetpoint;
            xSemaphoreGive(setpointMutex);
        }

        // --- Check if data changed enough to require redraw ---
        if (abs(tempToDisplay - prevTempToDisplay) > 0.1 ||
            pwmToDisplay != prevPwmToDisplay ||
            abs(setpointToDisplay - prevSetpointToDisplay) > 0.01 ||
            controlStateToDisplay != prevControlStateToDisplay) // Also check if control state changed
        {
            redrawRequired = true;
        }

        // --- Update blink state for menu cursor ---
        unsigned long currentTime = millis();
        if (currentTime - blinkStartTime > blinkInterval) {
            blinkState = !blinkState;
            blinkStartTime = currentTime;
            if (currentState == STATE_SETTINGS_MENU) { // Only force redraw for menu blinking
                 redrawRequired = true;
            }
        }

        // --- Draw only if required ---
        if (redrawRequired) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) { // Use shorter timeout
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB08_tr);
                u8g2.setDrawColor(1);

                // --- Draw based on State ---
                switch (currentState) {
                    case STATE_HOME:
                        if (homePageIndex == 0) {
                            // --- Page 0: Main Control (Reference Image Style) ---
                            int setpointBoxWidth = 38; // Adjust as needed
                            int setpointBoxHeight = 38;
                            int setpointBoxX = 0;
                            int setpointBoxY = 0;

                            // Draw Setpoint Box
                            u8g2.drawFrame(setpointBoxX, setpointBoxY, setpointBoxWidth, setpointBoxHeight);

                            // Display Setpoint Value inside box
                            u8g2.setFont(u8g2_font_helvB18_tr); // Medium Bold Font for Setpoint Num
                            dtostrf(setpointToDisplay, 3, 0, tempStr); // Format as integer
                            int setpointNumWidth = u8g2.getStrWidth(tempStr);
                            int setpointNumX = setpointBoxX + (setpointBoxWidth - setpointNumWidth) / 2;
                            int setpointNumY = setpointBoxY + u8g2.getAscent() + 5; // Adjust vertical position
                            u8g2.drawStr(setpointNumX, setpointNumY, tempStr);

                            // Display "SET" Label below number
                            u8g2.setFont(u8g2_font_helvR08_tr); // Smaller Regular Font for SET label
                            int setLabelWidth = u8g2.getStrWidth("SET");
                            int setLabelX = setpointBoxX + (setpointBoxWidth - setLabelWidth) / 2;
                            int setLabelY = setpointBoxY + setpointBoxHeight - 3; // Position at bottom
                            u8g2.drawStr(setLabelX, setLabelY, "SET");

                            // Display Large Current Temperature
                            // u8g2.setFont(u8g2_font_logisoso30_tr); // Example large font (Choose one available)
                            u8g2.setFont(u8g2_font_fub30_tr); // Another large numeric font option
                            dtostrf(tempToDisplay < -900 ? 999 : tempToDisplay , 3, 0, tempStr); // Format as integer, use 999 for error
                            int currentTempWidth = u8g2.getStrWidth(tempStr);
                            int currentTempX = setpointBoxX + setpointBoxWidth + 5; // Position to the right of the box
                            int currentTempY = u8g2.getAscent() + 5; // Align baseline (adjust as needed)
                            u8g2.drawStr(currentTempX, currentTempY, tempStr);

                            // Display Degree C Symbol
                            u8g2.setFont(u8g2_font_unifont_t_symbols); // Font with degree symbol
                            int degreeWidth = u8g2.getStrWidth("°");
                            int degreeX = currentTempX + currentTempWidth - degreeWidth + 2; // Adjust position
                            int degreeY = setpointBoxY + u8g2.getAscent() + 2; // Align with top numbers
                            u8g2.drawGlyph(degreeX, degreeY, 0xb0); // Degree symbol
                            u8g2.setFont(u8g2_font_ncenB10_tr); // Smaller font for 'C'
                            u8g2.drawStr(degreeX + degreeWidth -1 , degreeY, "C");

                            // Bottom Area
                            int bottomY = u8g2.getDisplayHeight() - 1;
                            int pwmBarHeight = 8;

                            // Display PWM Percentage (Bottom Left)
                            u8g2.setFont(u8g2_font_ncenB08_tr); // Font for PWM %
                            int pwmPercentage = (int)(pwmToDisplay * 100.0 / 255.0);
                            snprintf(statusBuffer, sizeof(statusBuffer), "%d%%", pwmPercentage);
                            int pwmPercWidth = u8g2.getStrWidth(statusBuffer);
                            u8g2.drawStr(2, bottomY - 1, statusBuffer);

                            // Display PWM Bar (Bottom Right)
                            int barX = pwmPercWidth + 8; // Start bar after percentage + spacing
                            int barWidth = u8g2.getDisplayWidth() - barX - 2; // Bar fills remaining width
                            int barY = bottomY - pwmBarHeight;
                            int filledWidth = (int)((float)pwmToDisplay / 255.0 * barWidth);
                            u8g2.drawFrame(barX, barY, barWidth, pwmBarHeight);
                            u8g2.drawBox(barX + 1, barY + 1, filledWidth - 2 > 0 ? filledWidth - 2 : 0, pwmBarHeight - 2);

                            // Page Indicator (Optional - can be removed if only one primary page)
                            // u8g2.setFont(u8g2_font_micro_tr);
                            // u8g2.drawStr(u8g2.getDisplayWidth() / 2 - 2 , bottomY, "1/2");
 
                        } else if (homePageIndex == 1) {
                            // --- Page 1: Sensors & Params ---
                            u8g2.setFont(u8g2_font_ncenB08_tr);
                            int yPos = u8g2.getAscent() + 1;
                            const int lineHeight = u8g2.getMaxCharHeight() + 2;

                            // SHT40 Temp & Humidity
                            snprintf(statusBuffer, sizeof(statusBuffer), "SHT T:%.1fC H:%.0f%%", g_sht40Temp < -900 ? NAN : g_sht40Temp, g_sht40Humidity < -900 ? NAN : g_sht40Humidity);
                            u8g2.drawStr(0, yPos, statusBuffer);
                            yPos += lineHeight;

                            // NTC Voltage
                            snprintf(statusBuffer, sizeof(statusBuffer), "NTC V: %.2fV", g_ntcVoltage < -900 ? NAN : g_ntcVoltage);
                            u8g2.drawStr(0, yPos, statusBuffer);
                            yPos += lineHeight;

                            // PID Parameters
                            snprintf(statusBuffer, sizeof(statusBuffer), "Kp:%.1f Ki:%.1f", Kp, Ki);
                            u8g2.drawStr(0, yPos, statusBuffer);
                            yPos += lineHeight;
                            snprintf(statusBuffer, sizeof(statusBuffer), "Kd:%.1f", Kd);
                            u8g2.drawStr(0, yPos, statusBuffer);

                            // Page Indicator
                            u8g2.setFont(u8g2_font_micro_tr);
                            u8g2.drawStr(u8g2.getDisplayWidth() / 2 - 2 , u8g2.getDisplayHeight() -1, "2/2");
                        }
                        break;
                    case STATE_SETTINGS_MENU: {
                        u8g2.setFont(u8g2_font_ncenB08_tr); // Reset font
                        u8g2.drawStr(0, u8g2.getAscent(), "Settings Menu");
                        u8g2.drawLine(0, u8g2.getAscent() + 2, SCREEN_WIDTH, u8g2.getAscent() + 2);
                        int yPos = u8g2.getAscent() * 2 + 4;
                        for (int i = 0; i < numMenuItems; ++i) {
                            if (i == selectedMenuItem) {
                                // Draw blinking cursor instead of box
                                if (blinkState) {
                                    u8g2.drawStr(0, yPos, ">"); // Blinking cursor
                                }
                                // Text remains white (default color 1)
                                u8g2.setDrawColor(1); // Ensure text color is 1
                            }
                            if (i == 0) { // Target Temp
                                dtostrf(setpointToDisplay, 4, 1, tempStr);
                                snprintf(statusBuffer, sizeof(statusBuffer), "%s: %s C", menuItems[i], tempStr);
                                u8g2.drawStr(10, yPos, statusBuffer); // Indent text slightly
                            } else {
                                u8g2.drawStr(10, yPos, menuItems[i]); // Indent text slightly
                            }
                            u8g2.setDrawColor(1); // Ensure draw color is back to white for next item
                            yPos += u8g2.getMaxCharHeight() + 4;
                        }
                        break;
                    }
                    case STATE_EDIT_SETPOINT: {
                        u8g2.setFont(u8g2_font_ncenB08_tr); // Reset font
                        u8g2.drawStr(0, u8g2.getAscent(), "Edit Target Temp");
                        u8g2.drawLine(0, u8g2.getAscent() + 2, SCREEN_WIDTH, u8g2.getAscent() + 2);
                        dtostrf(editingSetpoint, 5, 1, tempStr);
                        snprintf(statusBuffer, sizeof(statusBuffer), "[ %s C ]", tempStr);
                        int textWidth = u8g2.getStrWidth(statusBuffer);
                        u8g2.setFont(u8g2_font_ncenB10_tr); // Larger font for value
                        u8g2.drawStr(SCREEN_WIDTH/2 - textWidth/2, SCREEN_HEIGHT/2 + 5, statusBuffer);
                        u8g2.setFont(u8g2_font_micro_tr);
                        u8g2.drawStr(0, SCREEN_HEIGHT - 2, "Up/Dn: +/- 0.5 | OK: Save | Back: Cancel");
                        break;
                    }
                } // End Drawing State Switch

                u8g2.sendBuffer();

                prevTempToDisplay = tempToDisplay;
                prevPwmToDisplay = pwmToDisplay;
                prevSetpointToDisplay = setpointToDisplay;
                prevControlStateToDisplay = controlStateToDisplay; // Store previous control state

                xSemaphoreGive(i2cMutex);
                redrawRequired = false; // Reset flag after successful draw
                lastRedrawTime = millis(); // Update last redraw time
            } else {
                Serial.println("UI Task: Failed to take I2C Mutex!");
                // Don't reset redrawRequired, try again next cycle
            }
        } // End if(redrawRequired)

        vTaskDelay(pdMS_TO_TICKS(20)); // Reduce delay slightly, rely on rate limiting logic
    }
}


