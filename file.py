#!/usr/bin/env python3
"""
Smart AI-Based Water Level Detector
Raspberry Pi Zero 2W + HC-SR04 + Relay + SIM800L
"""

import time
import serial
import statistics
from collections import deque
from dataclasses import dataclass
from enum import Enum
import RPi.GPIO as GPIO

# ─────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────

@dataclass
class Config:
    # GPIO Pins
    TRIG_PIN: int = 23
    ECHO_PIN: int = 24
    RELAY_PIN: int = 17
    
    # Tank dimensions (cm)
    TANK_HEIGHT: int = 100          # Total tank height
    SENSOR_OFFSET: int = 5          # Distance from sensor to max water level
    
    # Thresholds (percentage)
    LOW_THRESHOLD: int = 20         # Turn pump ON below this
    HIGH_THRESHOLD: int = 80        # Turn pump OFF above this
    CRITICAL_LOW: int = 10          # Send SMS alert
    OVERFLOW_WARNING: int = 95      # Send overflow alert
    
    # AI/Filtering settings
    READING_SAMPLES: int = 5        # Readings per measurement
    HISTORY_SIZE: int = 50          # Historical readings for prediction
    ANOMALY_THRESHOLD: float = 15.0 # Max deviation from median (cm)
    
    # Timing
    MEASUREMENT_INTERVAL: int = 5   # Seconds between measurements
    SMS_COOLDOWN: int = 300         # Minimum seconds between SMS alerts
    
    # SMS settings
    PHONE_NUMBER: str = "+1234567890"  # Your phone number
    SERIAL_PORT: str = "/dev/serial0"
    BAUD_RATE: int = 9600


class WaterState(Enum):
    CRITICAL_LOW = "CRITICAL_LOW"
    LOW = "LOW"
    NORMAL = "NORMAL"
    HIGH = "HIGH"
    OVERFLOW = "OVERFLOW"


# ─────────────────────────────────────────────────────────────
# Ultrasonic Sensor
# ─────────────────────────────────────────────────────────────

class UltrasonicSensor:
    def __init__(self, trig_pin: int, echo_pin: int):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trig_pin, False)
        time.sleep(0.5)  # Sensor settle time
    
    def measure_distance(self) -> float | None:
        """Measure distance in cm. Returns None on timeout."""
        # Send trigger pulse
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)
        
        # Wait for echo start
        timeout = time.time() + 0.1
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None
        
        # Wait for echo end
        timeout = time.time() + 0.1
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None
        
        # Calculate distance (speed of sound = 34300 cm/s)
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * 34300) / 2
        
        return round(distance, 2)


# ─────────────────────────────────────────────────────────────
# AI Water Level Analyzer
# ─────────────────────────────────────────────────────────────

class WaterLevelAI:
    """Handles filtering, prediction, and anomaly detection."""
    
    def __init__(self, config: Config):
        self.config = config
        self.history = deque(maxlen=config.HISTORY_SIZE)
        self.timestamps = deque(maxlen=config.HISTORY_SIZE)
    
    def filter_readings(self, readings: list[float]) -> float | None:
        """Apply median filter to remove outliers."""
        valid = [r for r in readings if r is not None and 2 < r < 400]
        if len(valid) < 3:
            return None
        return statistics.median(valid)
    
    def is_anomaly(self, reading: float) -> bool:
        """Detect anomalous readings using historical data."""
        if len(self.history) < 5:
            return False
        
        median = statistics.median(self.history)
        return abs(reading - median) > self.config.ANOMALY_THRESHOLD
    
    def add_reading(self, distance: float) -> None:
        """Add a validated reading to history."""
        self.history.append(distance)
        self.timestamps.append(time.time())
    
    def predict_trend(self) -> str:
        """Predict water level trend using linear regression."""
        if len(self.history) < 10:
            return "INSUFFICIENT_DATA"
        
        recent = list(self.history)[-20:]
        n = len(recent)
        
        # Simple linear regression
        x_mean = (n - 1) / 2
        y_mean = sum(recent) / n
        
        numerator = sum((i - x_mean) * (y - y_mean) for i, y in enumerate(recent))
        denominator = sum((i - x_mean) ** 2 for i in range(n))
        
        if denominator == 0:
            return "STABLE"
        
        slope = numerator / denominator
        
        # Slope interpretation (negative slope = rising water)
        if slope < -0.5:
            return "RISING_FAST"
        elif slope < -0.1:
            return "RISING"
        elif slope > 0.5:
            return "FALLING_FAST"
        elif slope > 0.1:
            return "FALLING"
        return "STABLE"
    
    def estimate_time_to_threshold(self, current_level: float, target_level: float) -> int | None:
        """Estimate minutes until water reaches target level."""
        if len(self.history) < 10:
            return None
        
        recent = list(self.history)[-10:]
        time_span = self.timestamps[-1] - self.timestamps[-10]
        
        if time_span == 0:
            return None
        
        rate = (recent[-1] - recent[0]) / time_span  # cm per second
        
        if abs(rate) < 0.001:
            return None
        
        distance_to_target = current_level - target_level
        seconds = distance_to_target / rate
        
        if seconds < 0:
            return None
        
        return int(seconds / 60)


# ─────────────────────────────────────────────────────────────
# SIM800L SMS Module
# ─────────────────────────────────────────────────────────────

class SIM800L:
    def __init__(self, port: str, baud_rate: int):
        self.serial = serial.Serial(port, baud_rate, timeout=3)
        time.sleep(2)
        self._initialize()
    
    def _initialize(self) -> bool:
        """Initialize the GSM module."""
        commands = [
            ("AT", "OK"),
            ("AT+CMGF=1", "OK"),      # Text mode
            ("AT+CNMI=1,2,0,0,0", "OK") # SMS notifications
        ]
        
        for cmd, expected in commands:
            if not self._send_command(cmd, expected):
                print(f"GSM init failed at: {cmd}")
                return False
        return True
    
    def _send_command(self, command: str, expected: str, timeout: int = 3) -> bool:
        """Send AT command and check response."""
        self.serial.write((command + "\r\n").encode())
        time.sleep(0.5)
        
        end_time = time.time() + timeout
        response = ""
        
        while time.time() < end_time:
            if self.serial.in_waiting:
                response += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                if expected in response:
                    return True
        return False
    
    def send_sms(self, phone: str, message: str) -> bool:
        """Send an SMS message."""
        try:
            self.serial.write(f'AT+CMGS="{phone}"\r\n'.encode())
            time.sleep(0.5)
            self.serial.write(f"{message}\x1a".encode())
            time.sleep(3)
            
            response = self.serial.read(self.serial.in_waiting).decode(errors='ignore')
            return "+CMGS:" in response
        except Exception as e:
            print(f"SMS error: {e}")
            return False
    
    def close(self):
        self.serial.close()


# ─────────────────────────────────────────────────────────────
# Relay Controller
# ─────────────────────────────────────────────────────────────

class RelayController:
    def __init__(self, pin: int):
        self.pin = pin
        self.is_on = False
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.HIGH)  # Relay OFF (active-low)
    
    def turn_on(self) -> None:
        if not self.is_on:
            GPIO.output(self.pin, GPIO.LOW)
            self.is_on = True
            print("💧 Pump ON")
    
    def turn_off(self) -> None:
        if self.is_on:
            GPIO.output(self.pin, GPIO.HIGH)
            self.is_on = False
            print("🛑 Pump OFF")


# ─────────────────────────────────────────────────────────────
# Main Controller
# ─────────────────────────────────────────────────────────────

class WaterLevelController:
    def __init__(self, config: Config):
        self.config = config
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Initialize components
        self.sensor = UltrasonicSensor(config.TRIG_PIN, config.ECHO_PIN)
        self.relay = RelayController(config.RELAY_PIN)
        self.ai = WaterLevelAI(config)
        self.gsm = SIM800L(config.SERIAL_PORT, config.BAUD_RATE)
        
        self.last_sms_time = 0
        self.last_state = None
    
    def distance_to_percentage(self, distance: float) -> float:
        """Convert sensor distance to water level percentage."""
        effective_height = self.config.TANK_HEIGHT - self.config.SENSOR_OFFSET
        water_height = effective_height - (distance - self.config.SENSOR_OFFSET)
        percentage = (water_height / effective_height) * 100
        return max(0, min(100, round(percentage, 1)))
    
    def get_water_state(self, level: float) -> WaterState:
        """Determine water state from level percentage."""
        if level <= self.config.CRITICAL_LOW:
            return WaterState.CRITICAL_LOW
        elif level <= self.config.LOW_THRESHOLD:
            return WaterState.LOW
        elif level >= self.config.OVERFLOW_WARNING:
            return WaterState.OVERFLOW
        elif level >= self.config.HIGH_THRESHOLD:
            return WaterState.HIGH
        return WaterState.NORMAL
    
    def measure_water_level(self) -> tuple[float, float] | None:
        """Take multiple readings and return filtered distance and level."""
        readings = []
        for _ in range(self.config.READING_SAMPLES):
            reading = self.sensor.measure_distance()
            if reading:
                readings.append(reading)
            time.sleep(0.1)
        
        filtered_distance = self.ai.filter_readings(readings)
        if filtered_distance is None:
            return None
        
        # Check for anomaly
        if self.ai.is_anomaly(filtered_distance):
            print(f"⚠️ Anomaly detected: {filtered_distance}cm (ignored)")
            return None
        
        self.ai.add_reading(filtered_distance)
        level = self.distance_to_percentage(filtered_distance)
        
        return filtered_distance, level
    
    def should_send_sms(self) -> bool:
        """Check if enough time has passed since last SMS."""
        return (time.time() - self.last_sms_time) > self.config.SMS_COOLDOWN
    
    def send_alert(self, state: WaterState, level: float) -> None:
        """Send SMS alert for critical states."""
        if not self.should_send_sms():
            return
        
        trend = self.ai.predict_trend()
        
        messages = {
            WaterState.CRITICAL_LOW: f"🚨 CRITICAL: Water level at {level}%! Trend: {trend}",
            WaterState.OVERFLOW: f"🚨 OVERFLOW WARNING: Water at {level}%! Trend: {trend}",
        }
        
        if state in messages:
            if self.gsm.send_sms(self.config.PHONE_NUMBER, messages[state]):
                self.last_sms_time = time.time()
                print(f"📱 SMS sent: {state.value}")
    
    def control_pump(self, level: float, state: WaterState) -> None:
        """Smart pump control with hysteresis."""
        if state in (WaterState.CRITICAL_LOW, WaterState.LOW):
            self.relay.turn_on()
        elif state in (WaterState.HIGH, WaterState.OVERFLOW):
            self.relay.turn_off()
        # NORMAL state: maintain current pump state (hysteresis)
    
    def run(self) -> None:
        """Main control loop."""
        print("=" * 50)
        print("🌊 Smart Water Level Detector Started")
        print("=" * 50)
        
        try:
            while True:
                result = self.measure_water_level()
                
                if result is None:
                    print("❌ Measurement failed")
                    time.sleep(self.config.MEASUREMENT_INTERVAL)
                    continue
                
                distance, level = result
                state = self.get_water_state(level)
                trend = self.ai.predict_trend()
                
                # Display status
                print(f"\n📊 Distance: {distance}cm | Level: {level}% | "
                      f"State: {state.value} | Trend: {trend} | "
                      f"Pump: {'ON' if self.relay.is_on else 'OFF'}")
                
                # Control logic
                self.control_pump(level, state)
                
                # Send alerts for state changes
                if state != self.last_state:
                    self.send_alert(state, level)
                    self.last_state = state
                
                # Estimate time to thresholds
                if trend in ("FALLING", "FALLING_FAST"):
                    eta = self.ai.estimate_time_to_threshold(
                        distance, 
                        self.config.TANK_HEIGHT - (self.config.LOW_THRESHOLD / 100 * self.config.TANK_HEIGHT)
                    )
                    if eta:
                        print(f"⏱️ Est. time to low threshold: {eta} min")
                
                time.sleep(self.config.MEASUREMENT_INTERVAL)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Shutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """Clean up resources."""
        self.relay.turn_off()
        self.gsm.close()
        GPIO.cleanup()
        print("✅ Cleanup complete")


# ─────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    config = Config(
        PHONE_NUMBER="+1234567890",  # ← Change this
        TANK_HEIGHT=100,             # ← Adjust to your tank
    )
    
    controller = WaterLevelController(config)
    controller.run()
