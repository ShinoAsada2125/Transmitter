# Debugging Guide - PCF8575 & Servo Control Issues

## Changes Made

### 1. Enhanced PCF8575 Control
- Added `PCF_ACTIVE_LOW` flag (set to `true` by default) to centralize relay polarity
- Created `setExpanderPin()` helper that logs every write with verification
- PCF8575 initialization now reads back all pins to verify communication

### 2. Comprehensive Debugging
- **LoRa Command Reception**: Shows hex dump, byte count, and parsed fields
- **Command Execution**: Traces every step with device type, current state, and actions
- **Servo Control**: Shows attach/detach, angle changes, and state transitions
- **PCF8575 Writes**: Logs pin number, ON/OFF state, and reads back pin state
- **Safety Control**: Shows when tank-full condition forces outputs OFF

### 3. Safety System Enhancement
- When tank is full, continuously forces all expander outputs OFF (every 2 seconds)
- Prints reminder message every 10 seconds when tank is full

## Testing Steps

### Step 1: Verify PCF8575 Communication
**Upload transmitter code and open Serial Monitor at 115200 baud**

Look for this on startup:
```
📄 Initializing PCF8575 I/O Expander...
   ✓ begin() called
   Setting all pins to OUTPUT, HIGH (relays off)...
   ✓ PCF8575 initialized. Initial state: 0xABCD
   All pins should be HIGH (0xFFFF): ✅ CORRECT
```

**Expected**: State should be `0xFFFF` (all HIGH)  
**If different**: PCF8575 may not be responding or has different I2C address

### Step 2: Test Float Sensor  
**Trigger the float sensor (connect pin 16 to GND to simulate tank full)**

Expected serial output:
```
*** EMERGENCY SHUTDOWN: TANK FULL ***
🔧 setExpanderPin(0, OFF) -> writing HIGH
   PCF8575 all pins state: 0xFFFF (pin 0 = HIGH)
🔧 setExpanderPin(1, OFF) -> writing HIGH
   ...
```

**Check**:
- LCD should show "*** TANK FULL ***"
- All LEDs on PCF8575 should turn OFF
- Serial should show all pins being forced OFF

### Step 3: Test Commands from Flutter App
**With tank NOT full, send commands from Flutter app**

#### Test 3A: Turn ON a fan
1. Press "HEATER ON" button in Flutter app
2. Watch Serial Monitor on RECEIVER ESP32:
   ```
   📲 BLE RX: HEATER:ON
      Device: HEATER | Action: ON
   📤 LoRa TX COMMAND: [HEATER:ON]
      ✅ Command sent
   ```

3. Watch Serial Monitor on TRANSMITTER ESP32:
   ```
   📥 DIO0 HIGH - attempting to read LoRa data...
      📦 Raw bytes count: 10
      📝 Raw string: [HEATER:ON]
      📏 Length: 10 chars
      🔢 Hex: 48 45 41 54 45 52 3A 4F 4E
      ✅ Command queued for execution

   🎯 NEW COMMAND FLAG DETECTED!
      Command: [HEATER:ON]
      Tank Full: NO
      ⚙️ Executing command...
   
   ⚙️ EXECUTE COMMAND:
      Searching for device: [HEATER] in 5 mapped devices...
      ✓ MATCH!
      ✅ SUCCESS: HEATER (PCF8575 pin 3) -> ON
   
   🔧 setExpanderPin(3, ON) -> writing LOW
      PCF8575 all pins state: 0xFFF7 (pin 3 = LOW)
   
   📤 Sending command feedback...
      Response: ACKCMD:HEATER:OK
      ✓ Feedback sent successfully
   ```

**Expected**: Pin 3 LED should turn ON, feedback should appear in Flutter log

#### Test 3B: Turn OFF the dehumidifier (servo)
1. Press "DEHUM OFF" button in Flutter app
2. Watch for servo activity:
   ```
   🎮 SERVO COMMAND DETECTED:
      Command wants: OFF
      Current state: ON
      Servo pin: 25
      🔄 Toggling servo (pressing button)...
      1. Attaching servo to pin 25
      2. Moving to PRESS angle (60°)
      3. Moving to REST angle (0°)
      4. Detaching servo
      ✅ Servo toggle complete. Dehumidifier now: OFF
   ```

**Expected**: Servo should physically move, state should flip

### Step 4: Test Safety Override
**With tank full, try to turn something ON**

1. Connect float sensor (pin 16 to GND)
2. Send "HEATER:ON" from app
3. Expected output:
   ```
   ❌ REJECTED: HEATER cannot be enabled when tank is FULL
   ```

4. Every 2 seconds, safety system should force outputs OFF:
   ```
   ⚠️ SAFETY: Tank still full, all expander outputs forced OFF
   ```

## Troubleshooting

### Issue: PCF8575 initial state is not 0xFFFF
**Cause**: PCF8575 not responding or wrong I2C address  
**Fix**:
1. Check I2C wiring (SDA=21, SCL=22)
2. Try different address: change `PCF8575_ADDR` to `0x21` or scan I2C bus
3. Power cycle the board

### Issue: Commands not reaching transmitter
**Symptom**: No "DIO0 HIGH" message after sending command  
**Check**:
1. Receiver serial: Is "📤 LoRa TX COMMAND" shown?
2. Transmitter serial: Is LoRa in RX mode? (should see "✅ LoRa initialized")
3. LoRa frequency offset matches on both (FREQ_OFFSET = +7000)

### Issue: Servo not moving
**Check**:
1. Is servo physically connected to pin 25?
2. Is servo getting 5V power?
3. Does serial show servo commands with angle changes?
4. Try manually: `dehumServo.attach(25); dehumServo.write(90); delay(1000);`

### Issue: PCF8575 pins not changing
**Check serial for each `setExpanderPin()` call**:
- Does it show "writing LOW" or "writing HIGH"?
- Does "all pins state" change?
- If state doesn't change: I2C communication problem or relay wiring

### Issue: Relays stay ON instead of OFF
**Fix**: Change polarity flag:
```cpp
const bool PCF_ACTIVE_LOW = false;  // Change from true to false
```
Then reflash and test.

## Expected Normal Operation

1. **Startup**: All PCF8575 outputs HIGH (relays OFF), servo at rest, tank checks
2. **Commands**: Received via LoRa, executed with full feedback
3. **Tank Full**: All protected devices forced OFF, commands rejected
4. **Tank Empty**: Commands execute normally, devices controllable

## Serial Monitor Tips

- Use **115200 baud**
- Look for emoji indicators: 📥 (receive), 📤 (send), ✅ (success), ❌ (error)
- Check timestamps to see if commands are delayed
- Compare receiver and transmitter logs side-by-side
