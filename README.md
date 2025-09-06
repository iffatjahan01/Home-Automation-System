Instruments used:
1. LDR (Light Dependent Resistor) → Detects light intensity and decides if the system should activate in low light.
2. DHT11 Sensor → Measures temperature and humidity.
3. MQ-2 Gas Sensor → Detects harmful gases and triggers the buzzer if gas concentration exceeds the threshold.
4. Ultrasonic Sensor (HC-SR04) → Measures water tank distance and calculates water level percentage.
5. IR Proximity Sensor → Detects a hand near the dispenser to activate the water pump.
6. L298N Motor Driver → Controls both the fan (cooling) and pump (water dispensing).
7. Buzzer → Provides an alarm when gas is detected.
8. LCD Display (I2C 16x2) → Shows temperature, water level, fan status, and gas status.
9. Push Button → Acts as a reset/silence button for the buzzer.

When the LDR senses low light, the system activates. The DHT11 sensor continuously monitors temperature: if the temperature crosses the set threshold, the fan automatically turns on, and switches off when it falls below. The MQ-2 gas sensor monitors air quality; if gas concentration is equal to or above the threshold, the buzzer sounds an alarm, which can be silenced using the reset button. The ultrasonic sensor checks the distance inside the tank to estimate water level, and these readings are displayed on the LCD along with the temperature and fan/gas status. For water dispensing, the IR sensor detects a hand near the outlet, and only then the pump runs to dispense water. All sensor readings and system status are also printed in the Serial Monitor for debugging.
