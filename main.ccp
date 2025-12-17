/*
 * ESP32 Smart Fan Controller with FreeRTOS
 * Course: CSEN B601 - Embedded System Architecture
 * GIU Berlin - Winter Semester 2025
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// Pin Definitions
#define DHT_PIN 4           // GPIO4 - DHT22 Data Pin
#define FAN_PWM_PIN 25      // GPIO25 - PWM Output for Fan Control
#define BTN_MODE 26         // GPIO26 - Mode Switch Button
#define BTN_UP 27           // GPIO27 - Speed Increase Button
#define BTN_DOWN 14         // GPIO14 - Speed Decrease Button
#define OLED_SDA 21         // GPIO21 - I2C SDA
#define OLED_SCL 22         // GPIO22 - I2C SCL

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DHT22 Sensor Configuration
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// PWM Configuration
#define PWM_FREQ 25000      // 25 kHz PWM frequency
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)
// Note: PWM_CHANNEL is no longer needed in ESP32 Core v3.0+

// Temperature Thresholds for Automatic Mode
#define TEMP_MIN 20.0       // Minimum temperature (fan off)
#define TEMP_LOW 25.0       // Low temperature threshold
#define TEMP_MED 30.0       // Medium temperature threshold
#define TEMP_HIGH 35.0      // High temperature threshold
#define TEMP_MAX 40.0       // Maximum temperature (fan full speed)

// System States
enum OperationMode {
  MODE_AUTOMATIC,
  MODE_MANUAL
};

// Global Variables (Protected by Mutex)
float currentTemperature = 0.0;
int fanSpeed = 0;               // 0-100%
OperationMode systemMode = MODE_AUTOMATIC;
bool displayUpdate = true;

// FreeRTOS Handles
SemaphoreHandle_t xMutex;
QueueHandle_t xTemperatureQueue;
TaskHandle_t xTempTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xButtonTaskHandle = NULL;
TaskHandle_t xDisplayTaskHandle = NULL;

// Button Debouncing
unsigned long lastDebounceTime[3] = {0, 0, 0};
const unsigned long debounceDelay = 50;
bool lastButtonState[3] = {HIGH, HIGH, HIGH};
bool buttonState[3] = {HIGH, HIGH, HIGH};

// Function Prototypes
void TaskTemperatureRead(void *pvParameters);
void TaskFanControl(void *pvParameters);
void TaskButtonInput(void *pvParameters);
void TaskDisplay(void *pvParameters);
int mapTemperatureToSpeed(float temp);
void updateFanSpeed(int speed);
void initializeHardware();

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Smart Fan Controller Starting...");
  
  // Initialize Hardware
  initializeHardware();
  
  // Create Mutex for shared resource protection
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Failed to create Mutex!");
    while(1);
  }
  
  // Create Queue for temperature data
  xTemperatureQueue = xQueueCreate(5, sizeof(float));
  if (xTemperatureQueue == NULL) {
    Serial.println("Failed to create Queue!");
    while(1);
  }
  
  // Create FreeRTOS Tasks
  xTaskCreatePinnedToCore(
    TaskTemperatureRead,      // Task function
    "TempRead",               // Task name
    4096,                     // Stack size
    NULL,                     // Parameters
    2,                        // Priority (High)
    &xTempTaskHandle,         // Task handle
    0                         // Core 0
  );
  
  xTaskCreatePinnedToCore(
    TaskFanControl,
    "FanControl",
    4096,
    NULL,
    2,                        // Priority (High)
    &xControlTaskHandle,
    0
  );
  
  xTaskCreatePinnedToCore(
    TaskButtonInput,
    "ButtonInput",
    2048,
    NULL,
    1,                        // Priority (Medium)
    &xButtonTaskHandle,
    1                         // Core 1
  );
  
  xTaskCreatePinnedToCore(
    TaskDisplay,
    "Display",
    4096,
    NULL,
    0,                        // Priority (Low)
    &xDisplayTaskHandle,
    1
  );
  
  Serial.println("All tasks created successfully!");
  Serial.println("System initialized. Starting operation...");
}

void loop() {
  // Empty - FreeRTOS tasks handle everything
  vTaskDelay(portMAX_DELAY);
}

void initializeHardware() {
  // Initialize DHT22 Sensor
  dht.begin();
  Serial.println("DHT22 initialized");
  
  // --- FIXED PWM INITIALIZATION FOR ESP32 CORE v3.0 ---
  // New syntax: ledcAttach(Pin, Frequency, Resolution)
  ledcAttach(FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(FAN_PWM_PIN, 0);  // Start with fan off
  Serial.println("PWM initialized");
  
  // Initialize Buttons with Internal Pull-up
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  Serial.println("Buttons initialized");
  
  // Initialize OLED Display
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed - using Serial output");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Smart Fan");
    display.println("Controller");
    display.println("Initializing...");
    display.display();
    Serial.println("OLED initialized");
  }
  
  delay(1000);
}

// Task 1: Temperature Reading (Priority: High, Period: 500ms)
void TaskTemperatureRead(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(500);
  
  Serial.println("Temperature Task started");
  
  for(;;) {
    float temp = dht.readTemperature();
    
    if (isnan(temp)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      // Send temperature to queue
      if (xQueueSend(xTemperatureQueue, &temp, 0) != pdPASS) {
        Serial.println("Queue full, temperature data lost");
      }
      
      // Update global variable with mutex protection
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        currentTemperature = temp;
        xSemaphoreGive(xMutex);
      }
      
      Serial.print("Temperature Read: ");
      Serial.print(temp);
      Serial.println(" °C");
    }
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// Task 2: Fan Control (Priority: High, Period: 200ms)
void TaskFanControl(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(200);
  float receivedTemp = 0.0;
  int targetSpeed = 0;
  
  Serial.println("Fan Control Task started");
  
  for(;;) {
    // Try to receive temperature from queue
    if (xQueueReceive(xTemperatureQueue, &receivedTemp, 0) == pdPASS) {
      // Successfully received new temperature data
    }
    
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      if (systemMode == MODE_AUTOMATIC) {
        // Automatic mode: Calculate speed based on temperature
        targetSpeed = mapTemperatureToSpeed(receivedTemp);
        fanSpeed = targetSpeed;
      }
      // In manual mode, fanSpeed is already set by button task
      
      xSemaphoreGive(xMutex);
    }
    
    // Update PWM duty cycle
    updateFanSpeed(fanSpeed);
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// Task 3: Button Input (Priority: Medium, Period: 100ms)
void TaskButtonInput(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100);
  
  Serial.println("Button Task started");
  
  for(;;) {
    // Read button states
    int reading[3];
    reading[0] = digitalRead(BTN_MODE);
    reading[1] = digitalRead(BTN_UP);
    reading[2] = digitalRead(BTN_DOWN);
    
    // Debounce and process each button
    for (int i = 0; i < 3; i++) {
      if (reading[i] != lastButtonState[i]) {
        lastDebounceTime[i] = millis();
      }
      
      if ((millis() - lastDebounceTime[i]) > debounceDelay) {
        if (reading[i] != buttonState[i]) {
          buttonState[i] = reading[i];
          
          // Button pressed (active LOW with pull-up)
          if (buttonState[i] == LOW) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
              switch(i) {
                case 0: // Mode button
                  systemMode = (systemMode == MODE_AUTOMATIC) ? MODE_MANUAL : MODE_AUTOMATIC;
                  Serial.print("Mode changed to: ");
                  Serial.println((systemMode == MODE_AUTOMATIC) ? "AUTOMATIC" : "MANUAL");
                  displayUpdate = true;
                  break;
                  
                case 1: // Speed Up button (only in manual mode)
                  if (systemMode == MODE_MANUAL) {
                    fanSpeed = min(100, fanSpeed + 10);
                    Serial.print("Manual speed increased to: ");
                    Serial.println(fanSpeed);
                    displayUpdate = true;
                  }
                  break;
                  
                case 2: // Speed Down button (only in manual mode)
                  if (systemMode == MODE_MANUAL) {
                    fanSpeed = max(0, fanSpeed - 10);
                    Serial.print("Manual speed decreased to: ");
                    Serial.println(fanSpeed);
                    displayUpdate = true;
                  }
                  break;
              }
              xSemaphoreGive(xMutex);
            }
          }
        }
      }
      lastButtonState[i] = reading[i];
    }
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// Task 4: Display Update (Priority: Low, Period: 1000ms)
void TaskDisplay(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);
  
  Serial.println("Display Task started");
  
  for(;;) {
    float temp;
    int speed;
    OperationMode mode;
    
    // Read shared variables with mutex protection
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      temp = currentTemperature;
      speed = fanSpeed;
      mode = systemMode;
      xSemaphoreGive(xMutex);
    }
    
    // Update Serial Monitor
    Serial.println("===== System Status =====");
    Serial.print("Mode: ");
    Serial.println((mode == MODE_AUTOMATIC) ? "AUTOMATIC" : "MANUAL");
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    Serial.print("Fan Speed: ");
    Serial.print(speed);
    Serial.println(" %");
    Serial.println("========================");
    
    // Update OLED Display
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    
    display.println("Smart Fan Control");
    display.println("-------------------");
    
    display.print("Mode: ");
    display.println((mode == MODE_AUTOMATIC) ? "AUTO" : "MANUAL");
    
    display.println();
    display.setTextSize(2);
    display.print(temp, 1);
    display.println(" C");
    
    display.print(speed);
    display.println(" %");
    
    display.display();
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// Helper Function: Map Temperature to Fan Speed (0-100%)
int mapTemperatureToSpeed(float temp) {
  if (temp < TEMP_MIN) {
    return 0;  // Fan off
  } else if (temp < TEMP_LOW) {
    return map((int)(temp * 10), TEMP_MIN * 10, TEMP_LOW * 10, 0, 25);
  } else if (temp < TEMP_MED) {
    return map((int)(temp * 10), TEMP_LOW * 10, TEMP_MED * 10, 25, 50);
  } else if (temp < TEMP_HIGH) {
    return map((int)(temp * 10), TEMP_MED * 10, TEMP_HIGH * 10, 50, 75);
  } else if (temp < TEMP_MAX) {
    return map((int)(temp * 10), TEMP_HIGH * 10, TEMP_MAX * 10, 75, 100);
  } else {
    return 100;  // Maximum speed
  }
}

// Helper Function: Update Fan PWM
void updateFanSpeed(int speed) {
  // Convert percentage (0-100) to PWM duty cycle (0-255)
  int dutyCycle = map(speed, 0, 100, 0, 255);
  
  // --- FIXED PWM WRITE FOR ESP32 CORE v3.0 ---
  // New syntax: ledcWrite(Pin, Value)
  ledcWrite(FAN_PWM_PIN, dutyCycle);
}
