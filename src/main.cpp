/*
================================================================================
ESP32 WATER TANK MONITOR WITH LoRa TRANSMISSION
================================================================================
Author: Your Name
Date: [Current Date]
Hardware: ESP32, AHT20, HC-SR04, 20x4 LCD, SX1278 LoRa

================================================================================
CONFIGURATION NOTES:
================================================================================

TIMING PARAMETERS (All in milliseconds - MS):
-------------------------------------------------------------------------------
- SENSOR_READ_INTERVAL    = 2000      (2 seconds)    → How often to read sensors
- LORA_TX_INTERVAL        = 10000     (10 seconds)   → How often to transmit via LoRa
- DISPLAY_UPDATE_INTERVAL = 1000      (1 second)     → How often to update LCD
- STATUS_PRINT_INTERVAL   = 5000      (5 seconds)    → How often to print serial status

TO CHANGE TRANSMISSION TO EVERY HOUR:
-------------------------------------------------------------------------------
Change: const unsigned long LORA_TX_INTERVAL = 3600000;  // 1 hour = 60 * 60 * 1000 ms

OTHER ADJUSTABLE PARAMETER(s):
-------------------------------------------------------------------------------
1. TANK DIMENSIONS (cm) - Change for your tank:
   - TANK_WIDTH_CM, TANK_LENGTH_CM, TANK_HEIGHT_CM
   
2. LoRa SETTINGS - Match with receiver:
   - LORA_FREQUENCY (433, 868, 915 MHz based on region)
   - LORA_OUTPUT_POWER (2-20 dBm - lower saves power)
   - LORA_SPREADING_FACTOR (7-12 - higher = longer range, slower)
   
3. DISPLAY MODES - Toggle with button:
   - Mode 0: Temperature & Humidity
   - Mode 1: Water Volume & Percentage
================================================================================*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>

// Enable a quick debug transmit mode for testing (set to 1 to enable)
#define DEBUG_TX_TEST 0  // ← DISABLED FOR COMMAND TESTING (no sensor broadcasts)
// NOTE: Set back to 1 to re-enable 5-second sensor broadcasts
#include <Adafruit_AHTX0.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
#include <RadioLib.h>
#include <PCF8575.h> 

// ================================================================================
// SECTION 1: HARDWARE PIN CONFIGURATION (Change if wiring differs)
// ================================================================================
#define TRIG_PIN      27      // Was 27, now GPIO 32
#define ECHO_PIN      26      // Was 26, now GPIO 33 (freeing GPIO 26 for I2C)     // GPIO 26 - Ultrasonic ECHO (INPUT)
#define MAX_DISTANCE  400     // HC-SR04 maximum range in cm

#define I2C_SDA       21      // GPIO 21 - I2C Data line
#define I2C_SCL       22      // GPIO 22 - I2C Clock line
#define BUTTON_PIN    15       // GPIO 2  - Display toggle button (INPUT_PULLUP)

// new aht20 sensor for inside 
#define I2C2_SDA 32    // GPIO 32 for second I2C SDA
#define I2C2_SCL 33    // GPIO 33 for second I2C SCL

// LoRa Module SPI Pins (SX1278)
#define LORA_SCK   18      // GPIO18 - SPI Clock
#define LORA_MISO  19      // GPIO19 - SPI Master In Slave Out
#define LORA_MOSI  23      // GPIO23 - SPI Master Out Slave In
#define LORA_CS    5       // GPIO5  - Chip Select
#define LORA_RST   14      // GPIO14 - Reset
#define LORA_DIO0  4       // GPIO4  - Digital I/O 0 (interrupt pin)

// Define which GPIO pins are available for remote control
// Example: GPIO 12 for FAN, GPIO 13 for HEATER
#define CONTROL_GPIO_1 12
#define CONTROL_GPIO_2 13
#define FLOAT_SENSOR_PIN 16 // Float sensor pin. LOW = Tank FULL (Float UP).
// Add more as needed
// PCF8575 EXPANDER PINS - ADD THESE
#define PCF8575_ADDR  0x20   // I2C address of expander
#define EXP_FAN1_PIN  0       // Intake fan
#define EXP_FAN2_PIN  1       // Circulation fan  
#define EXP_FAN3_PIN  2       // Condenser fan
#define EXP_HEATER_PIN 3      // Heater relay

// ================================================================================
// SECTION 2: LCD CONFIGURATION
// ================================================================================
#define LCD_ADDR      0x27    // I2C address (try 0x3F if display doesn't work)
#define LCD_COLS      20      // 20 columns
#define LCD_ROWS      4       // 4 rows

// ================================================================================
// SECTION 3: TANK DIMENSIONS (Customize for your tank!)
// ================================================================================
const float TANK_WIDTH_CM = 20.0;    // Interior width in cm
const float TANK_LENGTH_CM = 30.0;   // Interior length in cm
const float TANK_HEIGHT_CM = 40.0;   // Total height in cm

// ================================================================================
// SECTION 4: TIMING PARAMETERS (Easily adjustable - all in milliseconds)
// ================================================================================
const unsigned long SENSOR_READ_INTERVAL = 2000;    // 2 seconds - Read AHT20 & ultrasonic
// 10 seconds default transmit interval; override for debug test
#if DEBUG_TX_TEST
const unsigned long LORA_TX_INTERVAL = 5000;       // 5 seconds for debug testing
#else
const unsigned long LORA_TX_INTERVAL = 10000;       // 10 seconds - Transmit via LoRa
#endif
                                                    // For 1 hour: 3600000
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000; // 1 second - Update LCD
const unsigned long STATUS_PRINT_INTERVAL = 5000;   // 5 seconds - Serial status report

// ================================================================================
// SECTION 5: LoRa RADIO CONFIGURATION (Must match receiver!)
// ================================================================================
#define LORA_FREQUENCY       433.0   // Frequency in MHz (433, 868, or 915 based on region)
#define LORA_BANDWIDTH       125.0   // Bandwidth in kHz (125, 250, or 500)
#define LORA_SPREADING_FACTOR 10      // Spreading factor (7-12, higher = longer range)
#define LORA_CODING_RATE      5      // Coding rate (5-8, higher = better error correction)
#define LORA_OUTPUT_POWER     17     // Output power in dBm (2-20, lower saves power)
#define LORA_PREAMBLE_LENGTH  16      // Preamble length (8-65535)
#define LORA_SYNC_WORD        0x12   // Sync word (must match receiver, 0x12 default)
#define FREQ_OFFSET           +7000  // Hz - MUST MATCH RECEIVER EXACTLY!

// ================================================================================
// SECTION 6: DISPLAY MODES ENUM
// ================================================================================
enum DisplayMode {
  DISPLAY_WATER_VOLUME = 0,
  DISPLAY_TEMP_HUMID = 1,
  DISPLAY_SENSOR_COMPARE = 2
};

// ================================================================================
// SECTION 7: GLOBAL OBJECT INSTANCES
// ================================================================================
Adafruit_AHTX0 aht;  
Adafruit_AHTX0 aht2;                            // AHT20 temperature/humidity sensor
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // Ultrasonic distance sensor
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS); // 20x4 LCD display
Module mod = Module(LORA_CS, LORA_DIO0, LORA_RST);
SX1278 radio = SX1278(&mod);  // LoRa radio module
PCF8575 pcf8575(PCF8575_ADDR); // PCF8575 I/O expander
// ================================================================================
// SECTION 8: SENSOR DATA VARIABLES
// ================================================================================
float temperature = 0;          // Current temperature in °C
float humidity = 0;             // Current humidity in %
float distance_cm = 0;          // Ultrasonic distance in cm
float water_level_cm = 0;       // Water level in tank (cm)
float water_volume_liters = 0;  // Water volume in liters
float water_percentage = 0;     // Water level percentage (0-100%)
float temperature2 = 0;       // Second sensor temperature
float humidity2 = 0;          // Second sensor humidity
// ================================================================================
// SECTION 9: SYSTEM STATE VARIABLES
// ================================================================================
DisplayMode currentDisplayMode = DISPLAY_TEMP_HUMID; // Current LCD display mode

// Command reception variables
bool newCommandReceived = false;
String receivedCommand = "";
unsigned long lastCommandTime = 0;

// Command deduplication (receiver sends each command twice for reliability)
String lastExecutedCommand = "";
unsigned long lastExecutedTime = 0;
const unsigned long DEDUP_WINDOW_MS = 5000;  // Ignore same command within 5 seconds

// TANK CONTROL VARIABLES
//bool tankFull = false; // True when float is UP (state LOW)

bool isTankFull = false; // True when float sensor is LOW (tank full)
// Button debouncing variables
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // Button debounce delay in ms

// Timing control variables
unsigned long lastSensorReadTime = 0;
unsigned long lastLoRaTxTime = 0;
unsigned long lastDisplayUpdateTime = 0;
unsigned long lastStatusPrintTime = 0;
//dehumidifer tracking
bool dehumidifierOn = false;
// LoRa status tracking
bool loraInitialized = false;       // LoRa module initialization status
bool loraTransmitting = false;      // Current transmission state
unsigned int packetCounter = 0;     // Number of packets transmitted
unsigned long loraLastTxSuccess = 0; // Timestamp of last successful transmission
unsigned long loraLastTxAttempt = 0; // Timestamp of last transmission attempt
int loraTxSuccessCount = 0;         // Count of successful transmissions
int loraTxErrorCount = 0;           // Count of failed transmissions
int lastRSSI = 0;                   // Last received signal strength indicator
float lastSNR = 0;                  // Last signal-to-noise ratio

// ================================================================================
// SECTION 10: FUNCTION PROTOTYPES
// ================================================================================
void readSensors();
void updateDisplay();
void toggleDisplayMode();
void initializeLCD();
bool initializeLoRa();
void sendLoRaData();
void scanI2CDevices(TwoWire *wire = &Wire);
void showSystemStatus(String status, int row = 0);
void handleButtonPress();
String createLoRaPacket();
void showLoraStatusOnLCD(String status, int row, int col);
void clearLoraStatusFromLCD();
void printLoraStatusToSerial();
void checkForLoRaCommand();
bool executeCommand(String command);
void sendCommandFeedback(String cmd, bool success, String reason);
void emergencyShutdownAll();
void updateSafetyControl();



// Servo for dehumidifier
#define SERVO_PIN 25
Servo dehumServo;
const int SERVO_REST_ANGLE = 0;
const int SERVO_PRESS_ANGLE = 60; // degrees - change if needed
//unsigned long SERVO_PRESS_MS = 800; // default press duration (ms)

// Device mapping
const bool PCF_ACTIVE_LOW = false; // Your LEDs/relays are active-HIGH
enum DeviceType { DT_GPIO=0, DT_EXPANDER=1, DT_SERVO=2 };
struct DeviceMap { const char* name; DeviceType type; int gpio; uint8_t expAddr; uint8_t expPin; bool protectedWhenFull; };

// Hardcoded mapping - edit and reflash to change mappings
DeviceMap deviceMap[] = {
  {"HEATER", DT_EXPANDER, 0, PCF8575_ADDR, EXP_HEATER_PIN, true},   // Changed to expander + protected
  {"FAN1", DT_EXPANDER, 0, PCF8575_ADDR, EXP_FAN1_PIN, true},       // Changed to expander + protected
  {"FAN2", DT_EXPANDER, 0, PCF8575_ADDR, EXP_FAN2_PIN, true},       // Added
  {"FAN3", DT_EXPANDER, 0, PCF8575_ADDR, EXP_FAN3_PIN, true},       // Added
  {"DEHUM", DT_SERVO, SERVO_PIN, 0, 0, true},                       // Already protected
};
const int deviceMapCount = sizeof(deviceMap)/sizeof(deviceMap[0]);
void updateSafetyControl();
void setExpanderPin(uint8_t pin, bool on);

// ================================================================================
// SECTION 11: SETUP FUNCTION - Runs once at startup
// ================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor to initialize
  
  Serial.println("\n" + String(millis() / 1000) + "s: ========================================");
  Serial.println("        ESP32 WATER TANK MONITOR");
  Serial.println("        with LoRa Transmission");
  Serial.println("========================================\n");
  
  Serial.println("Initializing system components...");
  Serial.println("────────────────────────────────");
  
  // Initialize button with internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("✓ Button initialized (GPIO 15, INPUT_PULLUP)");
  
  // Initialize SPI with custom pins (CRITICAL: Before LoRa init!)
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  Serial.println("✓ SPI initialized");
  
  // Initialize I2C bus for LCD and AHT20
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // ---------- PCF8575 EXPANDER INITIALISATION ----------   // <--- ADD THIS BLOCK



  // Initialize second I2C bus for ambient sensor
  Wire1.begin(I2C2_SDA, I2C2_SCL);
  Wire1.setClock(100000);
  

  // Scan for I2C devices
  scanI2CDevices();


  
  // Initialize LCD display
  initializeLCD();
  
  // Initialize AHT20 temperature/humidity sensor
  if (!aht.begin()) {
    Serial.println("❌ ERROR: Failed to initialize AHT20 sensor!");
    showSystemStatus("AHT20 Error!", 1);
    while (1) {
      Serial.println("System halted - check AHT20 connection");
      delay(1000);
    }
  }
  Serial.println("✓ AHT20 temperature/humidity sensor initialized");
  showSystemStatus("AHT20: OK", 1);
  
// Initialize second I2C bus for ambient sensor
Wire1.begin(I2C2_SDA, I2C2_SCL);
Wire1.setClock(100000); // 100kHz I2C speed

if (!aht2.begin(&Wire1)) {  // Note: &Wire1 specifies second I2C bus
  Serial.println("⚠️ WARNING: Failed to initialize second AHT20!");
  // Don't halt system - run with one sensor if needed
  showSystemStatus("AHT20-2: Fail", 2);
} else {
  Serial.println("✓ Second AHT20 initialized (Ambient/Chamber)");
  showSystemStatus("AHT20-2: OK", 2);
}


  // Initialize LoRa radio module
  Serial.println("\nInitializing LoRa radio module...");
  loraInitialized = initializeLoRa();
  
  // Display system ready message
  lcd.clear();
  showSystemStatus("System Ready!", 0);
  showSystemStatus("Mode: Temp/Humid", 1);
  showSystemStatus("Press BTN to toggle", 2);
  
  if (loraInitialized) {
    showSystemStatus("LoRa: Ready", 3);
    Serial.println("✅ SYSTEM READY - LoRa TRANSMITTER ACTIVE");
  } else {
    showSystemStatus("LoRa: Disabled", 3);
    Serial.println("⚠️ SYSTEM READY - LoRa DISABLED");
  }
  
  Serial.println("\n" + String(millis() / 1000) + "s: Starting main loop...");
  Serial.println("========================================\n");
  
  delay(2000); // Show ready message for 2 seconds
  lcd.clear(); // Clear for main display

  pinMode(CONTROL_GPIO_1, OUTPUT);
  digitalWrite(CONTROL_GPIO_1, LOW);
  pinMode(CONTROL_GPIO_2, OUTPUT);
  digitalWrite(CONTROL_GPIO_2, LOW);
  Serial.println("✓ Remote control GPIOs initialized (OFF)");

//___________________________________________________
// Initialize Float Sensor pin
pinMode(FLOAT_SENSOR_PIN, INPUT_PULLUP);
Serial.println("✓ Float sensor initialized (INPUT_PULLUP)");

// Initialize Safety Control GPIO
pinMode(CONTROL_GPIO_2, OUTPUT);
digitalWrite(CONTROL_GPIO_2, LOW); // Start with dehumidifier allowed to run (LOW)
Serial.println("✓ Safety control GPIO13 initialized (LOW)");

  // Initialize servo to rest position (attach briefly then detach)


  
  
 // ========== TURN ON DEHUMIDIFIER AT STARTUP ==========
  if (digitalRead(FLOAT_SENSOR_PIN) == LOW) {
    Serial.println("Startup: Tank FULL - dehumidifier stays OFF");
    dehumidifierOn = false;
  } else {
    Serial.println("Startup: Turning dehumidifier ON...");
    dehumServo.attach(SERVO_PIN);
    dehumServo.write(SERVO_PRESS_ANGLE);   // Press button
    delay(500);                            // Hold
    dehumServo.write(SERVO_REST_ANGLE);    // Release
    delay(100);
    dehumServo.detach();
    dehumidifierOn = true;                 // Now ON
    Serial.println("Dehumidifier initialized to ON");
  }
  // ===================================================
  
// ================================================================================
// PCF8575 INITIALIZATION (Using Working Diagnostic Pattern)
// ================================================================================
Serial.println("\n📟 Initializing PCF8575 I/O Expander...");
Serial.println("   (Using proven diagnostic pattern)");

// Force begin without checking return (matches working diagnostic)
pcf8575.begin();
Serial.println("   ✓ PCF8575 begin() called");

// Set all 16 pins as OUTPUT, HIGH (LEDs/relays OFF for active-low)
Serial.println("   Setting all pins to OUTPUT, HIGH...");
for (int i = 0; i < 16; i++) {
    pcf8575.pinMode(i, OUTPUT, HIGH);
}
Serial.println("   ✓ All pins configured");

// DIAGNOSTIC TEST: Blink pin 0 (like working diagnostic code)
Serial.println("\n🔍 Running diagnostic test on pin 0...");
for (int blink = 0; blink < 3; blink++) {
    Serial.printf("   Blink %d: LED ON (LOW)\n", blink + 1);
    pcf8575.digitalWrite(0, LOW);  // LED on (active-low)
    delay(300);
    
    Serial.printf("   Blink %d: LED OFF (HIGH)\n", blink + 1);
    pcf8575.digitalWrite(0, HIGH); // LED off
    delay(300);
}
Serial.println("   ✓ Diagnostic test complete");
Serial.println("   (If LED blinked 3 times, PCF8575 is working!)\n");

// Ensure all pins are HIGH (off) after test
for (int i = 0; i < 16; i++) {
    pcf8575.digitalWrite(i, HIGH);
}

}

// ================================================================================
// SECTION 12: MAIN LOOP - Runs continuously
// ================================================================================
unsigned long commandFeedbackStart = 0;  // Track feedback display time
bool commandFeedbackActive = false;       // Flag for active feedback display

void loop() {
  unsigned long currentTime = millis();
  
  // Handle button press for display mode toggle (debounced)
  handleButtonPress();
  
  // 1. CRITICAL: Check for incoming LoRa commands EVERY LOOP ITERATION
  //    This must happen frequently to catch incoming commands
  checkForLoRaCommand();
  
  // 2. Execute any received command IMMEDIATELY
  if (newCommandReceived) {
    Serial.println("\n🎯 NEW COMMAND FLAG DETECTED!");
    Serial.printf("   Command: [%s]\n", receivedCommand.c_str());
    Serial.printf("   Tank Full: %s\n", isTankFull ? "YES" : "NO");
    
    newCommandReceived = false; // Reset flag immediately
    
    // CRITICAL FIX: Ignore ACK packets from receiver to prevent infinite loop
    if (receivedCommand.startsWith("ACK:") || receivedCommand.startsWith("SENT:") || 
        receivedCommand.startsWith("FORWARDED:")) {
      Serial.println("   ℹ️ Ignoring ACK/status from receiver (loop prevention)");
    }
    // DEDUPLICATION: Ignore same command within 5-second window
    // (Receiver sends each command twice for reliability against half-duplex collisions)
    else if (receivedCommand == lastExecutedCommand && 
             (millis() - lastExecutedTime) < DEDUP_WINDOW_MS) {
      Serial.printf("   ℹ️ Ignoring duplicate [%s] (received %lums ago)\n", 
                    receivedCommand.c_str(), millis() - lastExecutedTime);
    } else {
      Serial.println("   ⚙️ Executing command...");
      // Execute the command
      bool success = executeCommand(receivedCommand);
      Serial.printf("   Result: %s\n", success ? "✅ SUCCESS" : "❌ FAILED");
      
      // Track for deduplication
      lastExecutedCommand = receivedCommand;
      lastExecutedTime = millis();
      
      // Start feedback display timer (instead of blocking wait)
      commandFeedbackActive = true;
      commandFeedbackStart = millis();
    }
  }
  
  // 2b. FEEDBACK DISPLAY: Show result without blocking (only 1-2 seconds, not 3)
  if (commandFeedbackActive) {
    if (millis() - commandFeedbackStart > 2000) {
      // Feedback timeout - clear the line
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      commandFeedbackActive = false;
    }
    // DO NOT block the loop - let other tasks continue!
  }
  
  // SAFETY FIRST: Check float sensor EVERY loop iteration (not tied to slow sensor reads)
  updateSafetyControl();
  
  // Read sensors at specified interval (SENSOR_READ_INTERVAL)
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    readSensors();
  }
  
  // Update LCD display at specified interval (DISPLAY_UPDATE_INTERVAL)
  if (currentTime - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdateTime = currentTime;
    updateDisplay();
  }
  
  // Transmit data via LoRa at specified interval (LORA_TX_INTERVAL)
  if (loraInitialized && (currentTime - lastLoRaTxTime >= LORA_TX_INTERVAL)) {
    lastLoRaTxTime = currentTime;
    sendLoRaData();
  }
  
  // Print system status to serial monitor at specified interval (STATUS_PRINT_INTERVAL)
  printLoraStatusToSerial();
  
  delay(10); // Small delay for system stability
}



// ================================================================================
// SECTION 13: SENSOR READING FUNCTIONS
// ================================================================================
void readSensors() {
  // Read temperature and humidity from AHT20
  sensors_event_t humidity_event, temp_event;
  if (aht.getEvent(&humidity_event, &temp_event)) {
    temperature = temp_event.temperature;
    humidity = humidity_event.relative_humidity;
  } else {
    Serial.println("ERROR: Failed to read AHT20 sensor!");
  }
  
  // Read second AHT20 (ambient/chamber conditions)
sensors_event_t humidity_event2, temp_event2;
if (aht2.getEvent(&humidity_event2, &temp_event2)) {
  temperature2 = temp_event2.temperature;
  humidity2 = humidity_event2.relative_humidity;
} else {
  Serial.println("ERROR: Failed to read second AHT20!");
}


  // Read ultrasonic sensor with averaging (3 readings)
  unsigned int totalDistance = 0;
  byte validReadings = 0;
  
  for (int i = 0; i < 3; i++) {
    unsigned int pingDistance = sonar.ping_cm();
    
    if (pingDistance > 0 && pingDistance <= MAX_DISTANCE) {
      totalDistance += pingDistance;
      validReadings++;
    }
    delay(50); // Short delay between readings
  }
  
  if (validReadings > 0) {
    distance_cm = totalDistance / validReadings;
    water_level_cm = TANK_HEIGHT_CM - distance_cm;
    
    // Clamp values to valid range
    if (water_level_cm < 0) water_level_cm = 0;
    if (water_level_cm > TANK_HEIGHT_CM) water_level_cm = TANK_HEIGHT_CM;
    
    // Calculate water volume and percentage
    water_volume_liters = (TANK_WIDTH_CM * TANK_LENGTH_CM * water_level_cm) / 1000.0;
    water_percentage = (water_level_cm / TANK_HEIGHT_CM) * 100.0;
  } else {
    Serial.println("WARNING: Ultrasonic sensor error - no valid readings!");
  }
  
  // Print sensor data to serial monitor for debugging
  Serial.printf("Temp: %.1f°C | Humid: %.1f%% | Level: %.1fcm (%.1f%%) | Volume: %.2fL\n",
                temperature, humidity, water_level_cm, water_percentage, water_volume_liters);
}

// ================================================================================
// SAFETY CONTROL: Float Sensor Master Control
// ================================================================================
void updateSafetyControl() {
  // Read float sensor: LOW = closed = tank FULL (NO switch behavior)
  bool floatClosed = (digitalRead(FLOAT_SENSOR_PIN) == LOW);
  
  // Debounce: require consistent reading for 300ms before acting
  static bool lastFloatReading = false;
  static unsigned long floatStableTime = 0;
  
  if (floatClosed != lastFloatReading) {
    lastFloatReading = floatClosed;
    floatStableTime = millis();  // Reset debounce timer on change
    return; // Don't act yet, wait for stable reading
  }
  
  // Only update state after 300ms of stable reading
  if (millis() - floatStableTime < 300) return;
  
  isTankFull = floatClosed;
  
  static bool wasFull = false; // Track state change
  static unsigned long lastSafetyPrint = 0;
  
  if (isTankFull && !wasFull) {
    // ============ TANK JUST BECAME FULL - EMERGENCY SHUTDOWN ============
    Serial.println("\n*** EMERGENCY SHUTDOWN: TANK FULL ***");
    
    // Immediate LCD warning
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("*** TANK FULL ***");
    lcd.setCursor(0, 1);
    lcd.print("EMERGENCY STOP");
    
    emergencyShutdownAll();
    wasFull = true;
    
    // Send TANK_FULL status to receiver so Flutter app updates immediately
    if (loraInitialized) {
      Serial.println("📤 Sending TANK_FULL status to receiver...");
      String statusMsg = "STATUS:TANK_FULL";
      radio.transmit(statusMsg);
      radio.startReceive();
    }
    
  } else if (!isTankFull && wasFull) {
    // ============ TANK NO LONGER FULL - AUTO RESTORE ALL DEVICES ============
    Serial.println("\n*** TANK NO LONGER FULL - AUTO RESTORING ALL DEVICES ***");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TANK OK - RESTORING");
    
    // Turn ON all PCF8575 expander devices (FAN1, FAN2, FAN3, HEATER)
    for (int i = 0; i < deviceMapCount; i++) {
      if (deviceMap[i].type == DT_EXPANDER) {
        setExpanderPin(deviceMap[i].expPin, true);  // Turn ON
        Serial.printf("  ✅ Restored: %s (pin %d) -> ON\n", deviceMap[i].name, deviceMap[i].expPin);
      }
    }
    
    // Turn ON dehumidifier via servo (if it was off)
    if (!dehumidifierOn) {
      Serial.println("  🔄 Restoring dehumidifier -> ON");
      dehumServo.attach(SERVO_PIN);
      dehumServo.write(SERVO_PRESS_ANGLE);
      delay(500);
      dehumServo.write(SERVO_REST_ANGLE);
      delay(100);
      dehumServo.detach();
      dehumidifierOn = true;
      Serial.println("  ✅ Dehumidifier restored to ON");
    }
    
    wasFull = false;
    
    // Send TANK_OK status to receiver so Flutter app updates immediately
    if (loraInitialized) {
      Serial.println("📤 Sending TANK_OK status to receiver...");
      String statusMsg = "STATUS:TANK_OK";
      radio.transmit(statusMsg);
      radio.startReceive();
    }
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ALL DEVICES ON");
    
  } else if (isTankFull) {
    // Periodically re-enforce forced OFF (every 2 seconds, not every loop)
    if (millis() - lastSafetyPrint > 2000) {
      for (int i = 0; i < deviceMapCount; i++) {
        if (deviceMap[i].type == DT_EXPANDER) {
          setExpanderPin(deviceMap[i].expPin, false);
        }
      }
      
      // Status print every 10 seconds
      static unsigned long lastSafetyMsg = 0;
      if (millis() - lastSafetyMsg > 10000) {
        Serial.println("\n⚠️ SAFETY: Tank still full, all expander outputs forced OFF");
        lastSafetyMsg = millis();
      }
      lastSafetyPrint = millis();
    }
  }
}

// EMERGENCY SHUTDOWN — called when tank is full
void emergencyShutdownAll() {
  Serial.println("🚨 emergencyShutdownAll() EXECUTING...");
  
  // Turn off all PCF8575 relays
  for (int i = 0; i < 16; i++) {
    setExpanderPin(i, false);
  }
  Serial.println("   ✓ All PCF8575 pins set to OFF");
  
  // ALWAYS press servo to toggle dehumidifier OFF (safety critical)
  // Even if dehumidifierOn is false, press anyway to be safe
  Serial.printf("   Servo press: dehumidifierOn was %s\n", dehumidifierOn ? "true" : "false");
  dehumServo.attach(SERVO_PIN);
  delay(50);  // Let servo attach settle
  dehumServo.write(SERVO_PRESS_ANGLE);
  Serial.printf("   Servo -> %d degrees (PRESS)\n", SERVO_PRESS_ANGLE);
  delay(600);  // Hold press longer for reliability
  dehumServo.write(SERVO_REST_ANGLE);
  Serial.printf("   Servo -> %d degrees (REST)\n", SERVO_REST_ANGLE);
  delay(150);
  dehumServo.detach();
  dehumidifierOn = false;
  
  Serial.println("   ✅ Emergency shutdown complete");
}





// ================================================================================
// SECTION 14: DISPLAY FUNCTIONS
// ================================================================================
void updateDisplay() {
  lcd.clear(); // Clear display before updating
  
  if (currentDisplayMode == DISPLAY_TEMP_HUMID) {
    // ===== DISPLAY MODE 1: Temperature and Humidity =====
    lcd.setCursor(0, 0);
    lcd.print("  TEMPERATURE  ");     // Row 0: Title
    lcd.setCursor(2, 1);
    lcd.print(temperature, 1);        // Row 1: Temperature value
    lcd.print((char)223);             // Degree symbol
    lcd.print("C");
    
    lcd.setCursor(0, 2);
    lcd.print("   HUMIDITY    ");     // Row 2: Title
    lcd.setCursor(2, 3);
    lcd.print(humidity, 1);           // Row 3: Humidity value
    lcd.print("%");
    
  } 
  else if (currentDisplayMode == DISPLAY_SENSOR_COMPARE) {
  // ===== DISPLAY MODE 3: Dual Sensor Comparison =====
    lcd.setCursor(0, 0);
    lcd.print("  TEMPERATURE  ");     // Row 0: Title
    lcd.setCursor(2, 1);
    lcd.print(temperature2, 1);        // Row 1: Temperature value
    lcd.print((char)223);             // Degree symbol
    lcd.print("C");
    
    lcd.setCursor(0, 2);
    lcd.print("   HUMIDITY    ");     // Row 2: Title
    lcd.setCursor(2, 3);
    lcd.print(humidity2, 1);           // Row 3: Humidity value
    lcd.print("%");
  

}

  else {
    // ===== DISPLAY MODE 2: Water Volume and Level =====
    lcd.setCursor(0, 0);
    lcd.print("  WATER VOLUME  ");    // Row 0: Title
    lcd.setCursor(2, 1);
    lcd.print(water_volume_liters, 1); // Row 1: Volume value
    lcd.print(" L");
    
    lcd.setCursor(0, 2);
    lcd.print("     LEVEL     ");     // Row 2: Title
    lcd.setCursor(2, 3);
    lcd.print(water_percentage, 1);   // Row 3: Percentage value
    lcd.print("%");
    
    // Visual water level indicator
    lcd.setCursor(14, 3);
    if (water_percentage > 80) {
      lcd.print("[F]");  // Full
    } else if (water_percentage > 40) {
      lcd.print("[M]");  // Medium
    } else if (water_percentage > 10) {
      lcd.print("[L]");  // Low
    } else {
      lcd.print("[E]");  // Empty
    }
  }
  
  // Display mode indicator (top-right corner)
  lcd.setCursor(17, 0);
  if (currentDisplayMode == DISPLAY_TEMP_HUMID) {
  lcd.print("TO");
  } else if (currentDisplayMode == DISPLAY_WATER_VOLUME) {
  lcd.print("V");
  } else {
  lcd.print("TI");  // For COMPARE mode
  }
  
  // ===== LoRa STATUS INDICATOR (Top-right, positions 16-18) =====
  /*lcd.setCursor(16, 0);
  if (loraTransmitting) {
    lcd.print("TX");      // Currently transmitting
  } else if (loraInitialized) {
    unsigned long timeSinceLastTx = (millis() - loraLastTxSuccess) / 1000;
    if (timeSinceLastTx < 30) {
      lcd.print("OK");    // Recent success (<30 seconds)
    } else if (timeSinceLastTx < 300) {
      lcd.print("--");    // Older success (30s-5min)
    } else if (loraLastTxSuccess > 0) {
      lcd.print("??");    // Very old/no recent success
    } else {
      lcd.print("RDY");   // Ready but never transmitted
    }
  } else {
    lcd.print("NO");      // LoRa not initialized
  }
  
  // ===== PACKET COUNTER (Bottom-right corner) =====
  lcd.setCursor(16, 3);
  lcd.print("#");
  if (packetCounter < 10) {
    lcd.print("00");
    lcd.print(packetCounter);
  } else if (packetCounter < 100) {
    lcd.print("0");
    lcd.print(packetCounter);
  } else {
    lcd.print(packetCounter % 1000);  // Show last 3 digits
  }*/
    // ===== TANK STATUS INDICATOR (Right side, row 1) =====
  lcd.setCursor(16, 1);  // Use columns 16-19 on row 1
  if (isTankFull) {
    lcd.print("FULL");
  } else {
    lcd.print("    ");  // Clear with 4 spaces when not full
  }

}

// Toggle between display modes

// In toggleDisplayMode() function, update to cycle through 3 modes:
void toggleDisplayMode() {
  currentDisplayMode = (DisplayMode)((currentDisplayMode + 1) % 3); // Changed from 2 to 3
  
  Serial.printf("Display mode changed to: %d\n", currentDisplayMode);
}

// Initialize LCD display
void initializeLCD() {
  // Try common I2C addresses for LCD
  uint8_t addresses[] = {0x27, 0x3F};
  bool lcdFound = false;
  
  for (uint8_t addr : addresses) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("LCD found at I2C address 0x%02X\n", addr);
      lcd = LiquidCrystal_I2C(addr, LCD_COLS, LCD_ROWS);
      lcdFound = true;
      break;
    }
  }
  
  if (!lcdFound) {
    Serial.println("ERROR: LCD not found on I2C bus! Check wiring.");
    return;
  }
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  Serial.println("✓ 20x4 LCD display initialized");
}

// ================================================================================
// SECTION 15: LoRa RADIO FUNCTIONS
// ================================================================================
bool initializeLoRa() {
  Serial.println("Initializing LoRa with frequency offset...");

  // Hardware reset (critical for stable operation)
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(100);
  digitalWrite(LORA_RST, HIGH);
  delay(200);

  // Calculate frequency with offset
  float freqWithOffset = LORA_FREQUENCY + (FREQ_OFFSET / 1e6);

  Serial.println("Initializing SX1278 LoRa module...");

  // Initialize LoRa module with specified parameters (matching receiver)
  int state = radio.begin(
    freqWithOffset,
    LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR,
    LORA_CODING_RATE,
    LORA_SYNC_WORD
  );

  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("❌ LoRa begin failed (code %d)\n", state);
    return false;
  }

  // CRITICAL FIX: Set CRC BEFORE starting receive/transmit
  radio.setCRC(true);

  // Set output power
  radio.setOutputPower(LORA_OUTPUT_POWER);
  Serial.printf("   Output power set to %d dBm\n", LORA_OUTPUT_POWER);

  // Start receive mode AFTER CRC and power configuration
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("❌ startReceive failed (code %d)\n", state);
    return false;
  }

  Serial.println("✅ LoRa initialized successfully!");
  Serial.printf("   Frequency: %.3f MHz (Offset: %d Hz)\n", freqWithOffset, FREQ_OFFSET);

  return true;
}

void sendLoRaData() {
  loraTransmitting = true;                     // Set transmission flag
  loraLastTxAttempt = millis();               // Record attempt time
  
  Serial.println("\n⚠️ ========== TRANSMITTER BUSY - NOT LISTENING ==========");
  
  String packet = createLoRaPacket();         // Create data packet
  
  Serial.println("\n" + String(millis() / 1000) + "s: ───────────────────");
  Serial.println("📡 LoRa TRANSMISSION ATTEMPT");
  Serial.println("──────────────────────────");
  Serial.printf("  Packet #%d\n", packetCounter);
  Serial.printf("  Length: %d bytes\n", packet.length());
  Serial.println("  Data: " + packet);
  
  // Show transmission status on LCD
  lcd.clear();
  showLoraStatusOnLCD("TX...", 0, 1);
  
  // Transmit packet via LoRa
  int state = radio.transmit(packet);
  
  // Return to RX mode explicitly (critical!)
  radio.startReceive();
  
  // Get signal quality metrics after transmission
  if (state == RADIOLIB_ERR_NONE) {
    lastRSSI = radio.getRSSI();  // Received Signal Strength Indicator
    lastSNR = radio.getSNR();    // Signal-to-Noise Ratio
  }
  
  if (state == RADIOLIB_ERR_NONE) {
    // ===== TRANSMISSION SUCCESS =====
    loraLastTxSuccess = millis();            // Record success time
    loraTxSuccessCount++;                    // Increment success counter
    packetCounter++;                         // Increment packet counter
    
    Serial.println("✅ LoRa TRANSMISSION SUCCESS!");
    Serial.printf("  RSSI: %d dBm\n", lastRSSI);
    Serial.printf("  SNR: %.1f dB\n", lastSNR);
    
    // Calculate and display success rate
    if (loraTxSuccessCount + loraTxErrorCount > 0) {
      float successRate = (float)loraTxSuccessCount / 
                         (loraTxSuccessCount + loraTxErrorCount) * 100;
      Serial.printf("  Success Rate: %.1f%% (%d/%d)\n", 
                    successRate, loraTxSuccessCount, 
                    loraTxSuccessCount + loraTxErrorCount);
    }
    
    Serial.println("──────────────────────────\n");
    
    // Show success on LCD for 2 seconds
    /*showLoraStatusOnLCD("✓ OK", 0, 1);
    
     // Display RSSI on bottom row
    String rssiStr = "RSSI:" + String(lastRSSI) + "dB";
    lcd.setCursor(0, 3);
    lcd.print("                    ");  // Clear line
    lcd.setCursor(0, 3);
    lcd.print(rssiStr);
    
    delay(2000); */ // Show success message for 2 seconds
    
  } else {
    // ===== TRANSMISSION FAILED =====
    loraTxErrorCount++;  // Increment error counter
    
    Serial.println("❌ LoRa TRANSMISSION FAILED!");
    Serial.printf("  Error Code: %d\n", state);
    Serial.println("  Possible causes:");
    
    // Display specific error message
    switch(state) {
      case RADIOLIB_ERR_PACKET_TOO_LONG:
        Serial.println("    - Packet too long (>256 bytes)");
        break;
      case RADIOLIB_ERR_TX_TIMEOUT:
        Serial.println("    - Transmission timeout");
        break;
      case RADIOLIB_ERR_SPI_WRITE_FAILED:
        Serial.println("    - SPI communication error");
        break;
      default:
        Serial.printf("    - Unknown error (code %d)\n", state);
    }
    
    Serial.printf("  Total Errors: %d\n", loraTxErrorCount);
    Serial.println("──────────────────────────\n");
    
    
  }
  
  loraTransmitting = false;  // Clear transmission flag
  
  Serial.println("✅ ========== BACK IN LISTENING MODE (can receive commands) ==========\n");

}

String createLoRaPacket() {
  // SIMPLIFIED FORMAT: "T1,H1,T2,H2,VOL,TANK"
  // Only 6 fields: temp1, humid1, temp2, humid2, water_volume, tank_full status
  // TANK: 1 = Full, 0 = Not Full
  
  String packet = String(temperature, 1) + ",";
  packet += String(humidity, 1) + ",";
  packet += String(temperature2, 1) + ",";
  packet += String(humidity2, 1) + ",";
  packet += String(water_volume_liters, 1) + ",";
  packet += String(isTankFull ? "1" : "0");
  
  return packet;
}

// ================================================================================
// NEW SECTION: COMMAND HANDLING FUNCTIONS
// ================================================================================

/**
 * Checks for incoming LoRa commands non-blockingly.
 * Call this frequently in loop().
 */
void checkForLoRaCommand() {
  // Method 1: Check DIO0 pin first (most reliable for SX1278)
  if (digitalRead(LORA_DIO0) == HIGH) {
    Serial.println("📥 DIO0 HIGH - attempting to read LoRa data...");
    
    uint8_t buf[256] = {0};  // Initialize to zero
    int state = radio.readData(buf, 255);  // Read up to 255 bytes

    if (state == RADIOLIB_ERR_NONE) {
      // Data received successfully
      buf[255] = 0;  // Ensure null-termination
      receivedCommand = String((char*)buf);
      receivedCommand.trim();  // Remove any whitespace
      
      Serial.printf("   📦 Raw bytes count: %d\n", strlen((char*)buf));
      Serial.printf("   📝 Raw string: [%s]\n", receivedCommand.c_str());
      Serial.printf("   📏 Length: %d chars\n", receivedCommand.length());
      
      // Show hex dump for first 20 chars
      if (receivedCommand.length() > 0) {
        Serial.print("   🔢 Hex: ");
        for (int i = 0; i < min(20, (int)receivedCommand.length()); i++) {
          Serial.printf("%02X ", (uint8_t)receivedCommand[i]);
        }
        Serial.println();
      }
      
      // Only process if command is not empty
      if (receivedCommand.length() > 0) {
        newCommandReceived = true;
        lastCommandTime = millis();
        
        Serial.println("   ✅ Command queued for execution in main loop");
        
        // Parse to show what we got
        int colonPos = receivedCommand.indexOf(':');
        if (colonPos > 0) {
          String device = receivedCommand.substring(0, colonPos);
          String action = receivedCommand.substring(colonPos + 1);
          device.trim();
          action.trim();
          Serial.printf("   ➡️ Parsed - Device: [%s] | Action: [%s]\n", device.c_str(), action.c_str());
        } else if (!receivedCommand.startsWith("ACK:") && !receivedCommand.startsWith("REJECT:")) {
          Serial.println("   ⚠️ Parse warning: No ':' separator found (not a valid command)");
        }
      } else {
        Serial.println("   ⚠️ Received empty data (ignoring)");
      }
      
      // Return to RX mode immediately
      radio.startReceive();
      return;
      
    } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
      // Some other error occurred
      Serial.printf("❌ readData error: %d\n", state);
      
      // Still try to restart receive
      radio.startReceive();
      
      // Small delay on error to prevent spam
      delay(100);
      return;
    }
    
    // RX_TIMEOUT is normal - just restart and continue
    radio.startReceive();
  }
}

/**
 * Parses and executes a command string.
 * Expected format: "DEVICE:ACTION" where ACTION is ON or OFF
 * Examples: "HEATER:ON", "FAN1:OFF", "DEHUM:ON"
 */
bool executeCommand(String cmd) {
  Serial.println("\n⚙️ EXECUTE COMMAND:");
  Serial.printf("  Input: [%s] (len=%d)\n", cmd.c_str(), cmd.length());
  cmd.trim(); // Remove whitespace
  Serial.printf("  After trim: [%s]\n", cmd.c_str());
  
  // Validate command is not empty
  if (cmd.length() == 0) {
    Serial.println("  ❌ ERROR: Empty command");
    sendCommandFeedback(cmd, false, "Empty command");
    return false;
  }
  
  // NEW FORMAT: DEVICE:ACTION (e.g., HEATER:ON, FAN1:OFF)
  int colonPos = cmd.indexOf(':');
  if (colonPos <= 0) {
    Serial.println("  ERROR: Invalid format - must be DEVICE:ACTION (e.g., HEATER:ON)");
    sendCommandFeedback(cmd, false, "Format error");
    return false;
  }
  
  String deviceName = cmd.substring(0, colonPos);
  String action = cmd.substring(colonPos + 1);
  deviceName.trim();
  action.trim();
  
  // Validate action is ON or OFF
  action.toUpperCase();
  if (action != "ON" && action != "OFF") {
    Serial.printf("  ERROR: Invalid action '%s' - must be ON or OFF\n", action.c_str());
    sendCommandFeedback(deviceName, false, "Invalid action (ON/OFF)");
    return false;
  }
  
  int val = (action == "ON") ? 1 : 0;
  
  // Find device in mapping
  Serial.printf("  Searching for device: [%s] in %d mapped devices...\n", deviceName.c_str(), deviceMapCount);
  for (int i = 0; i < deviceMapCount; i++) {
    Serial.printf("    Checking [%s] vs [%s]... ", deviceMap[i].name, deviceName.c_str());
    if (String(deviceMap[i].name) == deviceName) {
      Serial.println("✓ MATCH!");
      // Check if device is protected (e.g., dehumidifier when tank full)
      if (deviceMap[i].protectedWhenFull && isTankFull && val == 1) {
        Serial.printf("  ❌ REJECTED: %s cannot be enabled when tank is FULL\n", deviceName.c_str());
        sendCommandFeedback(deviceName, false, "Tank FULL");
        return false;
      }
      
      // Dispatch by device type
      if (deviceMap[i].type == DT_GPIO) {
        pinMode(deviceMap[i].gpio, OUTPUT);
        digitalWrite(deviceMap[i].gpio, val ? HIGH : LOW);
        Serial.printf("  ✅ SUCCESS: %s (GPIO %d) -> %s\n", 
                      deviceName.c_str(), deviceMap[i].gpio, (val ? "ON (HIGH)" : "OFF (LOW)"));
        sendCommandFeedback(deviceName, true, "OK");
        return true;
      } 
      else if (deviceMap[i].type == DT_EXPANDER) {
        setExpanderPin(deviceMap[i].expPin, val == 1);
        Serial.printf("  ✅ SUCCESS: %s (PCF8575 pin %d) -> %s\n", 
                      deviceName.c_str(), deviceMap[i].expPin, 
                      (val ? "ON" : "OFF"));
        sendCommandFeedback(deviceName, true, "OK");
        return true;
      }
      else if (deviceMap[i].type == DT_SERVO) {
        Serial.println("\n🎮 SERVO COMMAND DETECTED:");
        Serial.printf("   Command wants: %s\n", val ? "ON" : "OFF");
        Serial.printf("   Current state: %s\n", dehumidifierOn ? "ON" : "OFF");
        Serial.printf("   Servo pin: %d\n", SERVO_PIN);
        
        // Check if already in requested state
        if ((val == 1 && dehumidifierOn) || (val == 0 && !dehumidifierOn)) {
          Serial.println("   ℹ️ Already in requested state - no action needed");
          sendCommandFeedback(deviceName, true, "Already " + String(dehumidifierOn ? "ON" : "OFF"));
          return true;
        }
        
        // Need to toggle to reach desired state
        Serial.println("   🔄 Toggling servo (pressing button)...");
        Serial.printf("   1. Attaching servo to pin %d\n", SERVO_PIN);
        dehumServo.attach(SERVO_PIN);
        delay(50);
        
        Serial.printf("   2. Moving to PRESS angle (%d°)\n", SERVO_PRESS_ANGLE);
        dehumServo.write(SERVO_PRESS_ANGLE);
        delay(500);
        
        Serial.printf("   3. Moving to REST angle (%d°)\n", SERVO_REST_ANGLE);
        dehumServo.write(SERVO_REST_ANGLE);
        delay(100);
        
        Serial.println("   4. Detaching servo");
        dehumServo.detach();
        
        dehumidifierOn = !dehumidifierOn;  // State flipped
        
        Serial.printf("   ✅ Servo toggle complete. Dehumidifier now: %s\n", 
                      dehumidifierOn ? "ON" : "OFF");
        sendCommandFeedback(deviceName, true, dehumidifierOn ? "ON" : "OFF");
        return true;
      }
    }
  }
  
  // Device not found in mapping
  Serial.printf("  ❌ ERROR: Device '%s' not found in mapping!\n", deviceName.c_str());
  Serial.println("  Available devices:");
  for (int i = 0; i < deviceMapCount; i++) {
    Serial.printf("    - %s\n", deviceMap[i].name);
  }
  sendCommandFeedback(deviceName, false, "Device not found");
  return false;
}

// ================================================================================
// NEW SECTION FLOAT SENSOR HANDLING FUNCTIONS


// ================================================================================
// NEW SECTION FLOAT SENSOR HANDLING FUNCTIONS
// ================================================================================


// ================================================================================
// SECTION 16: LoRa STATUS DISPLAY FUNCTIONS
// ================================================================================
void showLoraStatusOnLCD(String status, int row, int col) {
  // Display temporary status message on LCD
  lcd.setCursor(col, row);
  lcd.print("     ");      // Clear previous status (5 characters)
  lcd.setCursor(col, row);
  lcd.print(status);       // Display new status
}

void clearLoraStatusFromLCD() {
  // Clear the temporary status area (columns 1-5, row 0)
  lcd.setCursor(1, 0);
  lcd.print("     ");
}

void printLoraStatusToSerial() {
  unsigned long now = millis();
  
  // Print status report at specified interval (STATUS_PRINT_INTERVAL)
  if (now - lastStatusPrintTime >= STATUS_PRINT_INTERVAL) {
    lastStatusPrintTime = now;
    
    Serial.println("\n📊 LoRa STATUS REPORT");
    Serial.println("══════════════════════════════");
    Serial.printf("  Module: %s\n", loraInitialized ? "INITIALIZED ✓" : "NOT INITIALIZED ✗");
    Serial.printf("  Current State: %s\n", loraTransmitting ? "TRANSMITTING..." : "IDLE");
    
    if (loraInitialized) {
      // Calculate time since last transmission
      unsigned long timeSinceLastTx = 0;
      if (lastLoRaTxTime > 0) {
        timeSinceLastTx = (now - lastLoRaTxTime) / 1000;
      }
      Serial.printf("  Last TX Attempt: %lu seconds ago\n", timeSinceLastTx);
      
      // Calculate time since last successful transmission
      if (loraLastTxSuccess > 0) {
        unsigned long timeSinceLastSuccess = (now - loraLastTxSuccess) / 1000;
        Serial.printf("  Last Success: %lu seconds ago\n", timeSinceLastSuccess);
      }
      
      // Display transmission statistics
      Serial.printf("  Statistics: %d OK, %d ERR\n", loraTxSuccessCount, loraTxErrorCount);
      
      // Calculate and display success rate
      if (loraTxSuccessCount + loraTxErrorCount > 0) {
        float successRate = (float)loraTxSuccessCount / 
                           (loraTxSuccessCount + loraTxErrorCount) * 100;
        Serial.printf("  Success Rate: %.1f%%\n", successRate);
      }
      
      // Display signal quality metrics
      Serial.printf("  Last RSSI: %d dBm\n", lastRSSI);
      Serial.printf("  Last SNR: %.1f dB\n", lastSNR);
      
      // Calculate time until next transmission
      if (lastLoRaTxTime > 0) {
        int secondsToNextTx = max(0, (int)(LORA_TX_INTERVAL - (now - lastLoRaTxTime)) / 1000);
        Serial.printf("  Next TX in: %d seconds\n", secondsToNextTx);
      }
    }
    Serial.println("══════════════════════════════\n");
  }
}

// ================================================================================
// SECTION 17: UTILITY FUNCTIONS
// ================================================================================
void scanI2CDevices(TwoWire *wire) {
  Serial.println("Scanning I2C bus for devices...");
  
  byte error, address;
  int deviceCount = 0;
  
  for (address = 1; address < 127; address++) {
    wire->beginTransmission(address);   // use the passed wire
    error = wire->endTransmission();
    
    if (error == 0) {
      Serial.printf("  Device found at address 0x%02X\n", address);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("  No I2C devices found! Check wiring.");
  } else {
    Serial.printf("  Found %d I2C device(s)\n", deviceCount);
  }
}


void showSystemStatus(String status, int row) {
  // Display system status message on LCD
  lcd.setCursor(0, row);
  lcd.print("                    ");  // Clear the entire row (20 spaces)
  lcd.setCursor(0, row);
  lcd.print(status);                  // Display status message
}

void handleButtonPress() {
  // Read current button state
  int reading = digitalRead(BUTTON_PIN);
  
  // Check for button state change (for debouncing)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // Reset debounce timer
  }
  
  // Apply debounce delay (debounceDelay = 50ms)
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      // Button is active LOW (pressed = LOW)
      if (currentButtonState == LOW) {
        toggleDisplayMode();  // Change display mode
      }
    }
  }
  
  lastButtonState = reading;  // Save current state for next comparison
}

// ================================================================================
// NEW: COMMAND FEEDBACK FUNCTION (ACK/NACK Response to Receiver)
// ================================================================================
void sendCommandFeedback(String device, bool success, String reason) {
  // Build ACK/NACK response in format: "ACKCMD:device:reason" or "REJECT:device:reason"
  String response;
  if (success) {
    response = "ACKCMD:" + device + ":" + reason;
  } else {
    response = "REJECT:" + device + ":" + reason;
  }
  
  // Try to transmit feedback back to receiver
  if (loraInitialized) {
    Serial.println("\n📤 Sending command feedback...");
    Serial.println("  Response: " + response);
    
    int state = radio.transmit(response);
    radio.startReceive();  // Return to RX mode
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("  ✓ Feedback sent successfully");
    } else {
      Serial.printf("  ✗ Feedback failed (code %d)\n", state);
    }
  }
}

// Wrapper to keep PCF8575 polarity in one place with readback verification
void setExpanderPin(uint8_t pin, bool on) {
  uint8_t level = on ? (PCF_ACTIVE_LOW ? LOW : HIGH)
                     : (PCF_ACTIVE_LOW ? HIGH : LOW);
  Serial.printf("🔧 setExpanderPin(%d, %s) -> writing %s\n", 
                pin, on ? "ON" : "OFF", level == LOW ? "LOW" : "HIGH");
  
  // Write to pin
  pcf8575.digitalWrite(pin, level);
  
  // Readback verification (using digitalRead for individual pin)
  delay(10); // Small delay for I2C to settle
  uint8_t readBack = pcf8575.digitalRead(pin);
  Serial.printf("   Readback: pin %d = %s (%s)\n", 
                pin, 
                readBack == HIGH ? "HIGH" : "LOW",
                readBack == level ? "✓ Match" : "✗ Mismatch");
}