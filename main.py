# Enhanced eBike Control System with Customizations
from machine import Pin, PWM, ADC, I2C, SPI, UART, CAN, Timer
import time
import math
from ssd1306 import SSD1306_I2C  # For OLED display
import ustruct  # For binary protocol communications

# Constants
MOTOR_250W = 0
MOTOR_500W = 1
MOTOR_1000W = 2

# Display Types
DISPLAY_NONE = 0
DISPLAY_OLED_I2C = 1
DISPLAY_LCD_SPI = 2
DISPLAY_TFT = 3

# Motor Controller Types
CTRL_PWM = 0
CTRL_UART = 1
CTRL_CAN = 2
CTRL_VESC = 3

class Config:
    def __init__(self):
        # Motor configuration
        self.motor_power = MOTOR_500W
        self.motor_controller_type = CTRL_UART  # Default to UART
        self.max_current = self._get_max_current()
        self.max_speed = self._get_max_speed()
        self.wheel_diameter = 0.7  # meters
        self.battery_cells = 13 if self.motor_power == MOTOR_250W else (
            16 if self.motor_power == MOTOR_500W else 20)
        
        # Display configuration
        self.display_type = DISPLAY_OLED_I2C
        
        # Features
        self.has_torque_sensor = True
        self.has_regenerative_braking = True
        
    def _get_max_current(self):
        if self.motor_power == MOTOR_250W:
            return 10  # Amps
        elif self.motor_power == MOTOR_500W:
            return 15  # Amps
        else:  # 1000W
            return 25  # Amps
            
    def _get_max_speed(self):
        if self.motor_power == MOTOR_250W:
            return 25  # km/h
        elif self.motor_power == MOTOR_500W:
            return 32  # km/h
        else:  # 1000W
            return 45  # km/h

class EBikeSystem:
    def __init__(self):
        # Initialize configuration
        self.config = Config()
        
        # Initialize hardware
        self._init_pins()
        self._init_motor_controller()
        self._init_display()
        self._init_sensors()
        
        # System state
        self.speed = 0  # km/h
        self.cadence = 0  # RPM
        self.torque = 0  # Nm (if torque sensor available)
        self.battery_level = 100  # %
        self.motor_temp = 25  # °C
        self.controller_temp = 25  # °C
        self.motor_power = 0  # Watts
        self.distance = 0  # meters
        self.lights_on = False
        self.horn_on = False
        self.assist_level = 1  # 1-3 (eco, normal, sport)
        self.charging = False
        self.regenerative_braking = False
        self.last_brake_time = 0
        
        # Timers
        self.update_timer = Timer(-1)
        self.update_timer.init(period=100, mode=Timer.PERIODIC, callback=self._update_system)
        
        # Safety limits
        self.max_motor_temp = 80  # °C
        self.max_controller_temp = 70  # °C
        self.low_battery_threshold = 20  # %
        self.critical_battery_threshold = 10  # %
        self.max_regen_current = 5  # Amps (for regenerative braking)
    
    def _init_pins(self):
        # Throttle (Analog input)
        self.throttle_adc = ADC(Pin(34))
        self.throttle_adc.atten(ADC.ATTN_11DB)
        
        # Brake sensors
        self.brake_front = Pin(12, Pin.IN, Pin.PULL_UP)
        self.brake_rear = Pin(13, Pin.IN, Pin.PULL_UP)
        
        # Cadence sensor (hall effect)
        self.cadence_sensor = Pin(14, Pin.IN, Pin.PULL_UP)
        self.cadence_last_time = time.ticks_ms()
        self.cadence_pulses = 0
        
        # Torque sensor (if available)
        if self.config.has_torque_sensor:
            self.torque_adc = ADC(Pin(36))
            self.torque_adc.atten(ADC.ATTN_11DB)
        
        # Lights control
        self.headlight = Pin(25, Pin.OUT)
        self.taillight = Pin(26, Pin.OUT)
        
        # Horn
        self.horn = Pin(27, Pin.OUT)
        
        # Battery monitoring
        self.battery_adc = ADC(Pin(35))
        self.battery_adc.atten(ADC.ATTN_11DB)
        
        # Temperature sensors
        self.motor_temp_adc = ADC(Pin(32))
        self.controller_temp_adc = ADC(Pin(33))
        
        # Charging detection
        self.charging_pin = Pin(5, Pin.IN, Pin.PULL_UP)
    
    def _init_motor_controller(self):
        """Initialize the appropriate motor controller interface"""
        self.motor_controller = None
        
        if self.config.motor_controller_type == CTRL_PWM:
            # Simple PWM control
            self.motor_pwm = PWM(Pin(15), freq=20000, duty=0)
        elif self.config.motor_controller_type == CTRL_UART:
            # UART communication (common for many controllers)
            self.motor_uart = UART(2, baudrate=9600, tx=16, rx=17)
        elif self.config.motor_controller_type == CTRL_CAN:
            # CAN bus communication
            try:
                self.motor_can = CAN(0, tx=5, rx=4, extframe=True, mode=CAN.NORMAL)
                self.motor_can.initfilter(0, CAN.LIST16, 0, (123, 124, 125, 126))  # Example filter
            except Exception as e:
                print("CAN init failed:", e)
        elif self.config.motor_controller_type == CTRL_VESC:
            # VESC-specific initialization
            self.motor_uart = UART(2, baudrate=115200, tx=16, rx=17)
            # VESC requires specific packet protocol
    
    def _init_display(self):
        """Initialize the selected display type"""
        self.display = None
        
        if self.config.display_type == DISPLAY_OLED_I2C:
            # I2C OLED display (128x64)
            self.i2c = I2C(scl=Pin(22), sda=Pin(21))
            self.display = SSD1306_I2C(128, 64, self.i2c)
        elif self.config.display_type == DISPLAY_LCD_SPI:
            # SPI LCD display
            self.spi = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(19), miso=Pin(23)))
            self.display_cs = Pin(15, Pin.OUT)
            # Initialize your specific LCD here
        elif self.config.display_type == DISPLAY_TFT:
            # TFT display with more advanced graphics
            # Initialize your specific TFT here
            pass
    
    def _init_sensors(self):
        # Initialize sensor interrupts
        self.cadence_sensor.irq(trigger=Pin.IRQ_RISING, handler=self._cadence_isr)
    
    def _cadence_isr(self, pin):
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, self.cadence_last_time)
        self.cadence_last_time = now
        
        if elapsed > 100:  # Debounce
            self.cadence_pulses += 1
            # Calculate RPM based on time between pulses (20 magnets = 1 revolution)
            if elapsed > 0:
                self.cadence = 60000 / (elapsed * 20)  # RPM
    
    def _update_system(self, timer):
        """Main system update called every 100ms"""
        # Read all sensors
        self._read_throttle()
        self._read_brakes()
        self._read_battery()
        self._read_temperatures()
        self._check_charging()
        if self.config.has_torque_sensor:
            self._read_torque()
        
        # Calculate speed based on cadence and wheel size
        self.speed = (self.cadence * 60 * math.pi * self.config.wheel_diameter) / 1000  # km/h
        
        # Update distance
        self.distance += (self.speed / 3.6) * 0.1  # 100ms update
        
        # Control motor based on inputs
        self._control_motor()
        
        # Update display if available
        if self.display is not None:
            self._update_display()
        
        # Check safety limits
        self._check_safety()
    
    def _read_throttle(self):
        throttle_raw = self.throttle_adc.read()
        # Convert to 0-100% (assuming 0-3.3V range)
        self.throttle = min(max((throttle_raw / 4095) * 100, 0), 100)
    
    def _read_brakes(self):
        brake_active = not self.brake_front.value() or not self.brake_rear.value()
        
        # Only update state if changed to avoid flickering
        if brake_active != self.brake_active:
            self.brake_active = brake_active
            self.last_brake_time = time.ticks_ms()
            
            # If regenerative braking is enabled and we just applied brakes
            if brake_active and self.config.has_regenerative_braking:
                self._activate_regenerative_braking(True)
            elif not brake_active and self.regenerative_braking:
                self._activate_regenerative_braking(False)
    
    def _read_torque(self):
        torque_raw = self.torque_adc.read()
        # Convert to torque value (Nm) - this requires calibration
        # Example conversion - adjust based on your sensor
        self.torque = (torque_raw / 4095) * 100  # 0-100Nm range
    
    def _read_battery(self):
        voltage_raw = self.battery_adc.read()
        # Convert to voltage (voltage divider ratio 1:5)
        voltage = (voltage_raw / 4095) * 3.3 * 6
        
        # Simple battery level calculation (adjust based on your battery)
        if voltage > (3.7 * self.config.battery_cells):
            self.battery_level = 100
        elif voltage < (3.0 * self.config.battery_cells):
            self.battery_level = 0
        else:
            self.battery_level = int((voltage - (3.0 * self.config.battery_cells)) / 
                                 ((3.7 - 3.0) * self.config.battery_cells) * 100)
    
    def _read_temperatures(self):
        # Read NTC thermistors (10k) - adjust based on your sensors
        motor_raw = self.motor_temp_adc.read()
        ctrl_raw = self.controller_temp_adc.read()
        
        # Simple conversion (for actual use, implement proper NTC conversion)
        self.motor_temp = motor_raw / 4095 * 100
        self.controller_temp = ctrl_raw / 4095 * 100
    
    def _check_charging(self):
        self.charging = not self.charging_pin.value()
    
    def _activate_regenerative_braking(self, activate):
        """Enable or disable regenerative braking"""
        if not self.config.has_regenerative_braking:
            return
            
        self.regenerative_braking = activate
        
        if activate:
            # Only activate if battery isn't almost full
            if self.battery_level < 95:
                print("Activating regenerative braking")
                # Implementation depends on motor controller
                if self.config.motor_controller_type == CTRL_VESC:
                    self._send_vesc_command(b'\x00\x00\x00\x00')  # Example VESC regen command
                elif self.config.motor_controller_type == CTRL_CAN:
                    # Send CAN message for regen braking
                    msg = bytearray(8)
                    msg[0] = 0x01  # Regen command
                    msg[1] = min(self.max_regen_current, 255)
                    self.motor_can.send(msg, 0x123)  # Example CAN ID
        else:
            print("Deactivating regenerative braking")
            # Send command to disable regen
            if self.config.motor_controller_type == CTRL_VESC:
                self._send_vesc_command(b'\x00\x00\x00\x00')  # Example VESC normal mode
    
    def _send_vesc_command(self, data):
        """Helper method to send VESC protocol commands"""
        if self.config.motor_controller_type != CTRL_VESC:
            return
            
        # Simple VESC packet structure (simplified)
        packet = bytearray()
        packet.append(0x02)  # Start byte
        packet.append(len(data))
        packet.extend(data)
        
        # Calculate CRC (simplified)
        crc = sum(data) & 0xFF
        packet.append(crc)
        packet.append(0x03)  # End byte
        
        self.motor_uart.write(packet)
    
    def _control_motor(self):
        """Control motor based on controller type"""
        if self.brake_active or self.battery_level < self.critical_battery_threshold:
            # Cut power if brakes are applied or critical battery level
            self._set_motor_power(0)
            return
        
        # Calculate target power based on throttle, torque, and assist level
        base_power = self.throttle * self.config.max_current / 100
        
        # Apply torque sensor input if available
        if self.config.has_torque_sensor and self.torque > 5:  # If pedaling with significant torque
            torque_factor = min(self.torque / 50, 2.0)  # Normalize to 50Nm
            base_power = base_power * (1 + torque_factor * 0.8)  # Add up to 80% more power
        
        # Apply assist level multiplier
        if self.assist_level == 1:  # Eco
            power = base_power * 0.7
        elif self.assist_level == 2:  # Normal
            power = base_power
        else:  # Sport
            power = base_power * 1.3
        
        # Add pedal assist if cadence is detected
        if self.cadence > 10:  # If pedaling
            cadence_factor = min(self.cadence / 60, 1.5)  # Normalize to 60RPM
            power = power * (1 + cadence_factor * 0.5)  # Add up to 50% more power
        
        # Limit power to motor rating
        power = min(power, self.config.max_current)
        
        # Set motor power based on controller type
        self._set_motor_power(power)
        
        # Calculate actual motor power (simplified)
        self.motor_power = (power / self.config.max_current) * (
            250 if self.config.motor_power == MOTOR_250W else 
            500 if self.config.motor_power == MOTOR_500W else 1000)
    
    def _set_motor_power(self, power):
        """Set motor power using the appropriate controller interface"""
        if self.config.motor_controller_type == CTRL_PWM:
            # Convert to PWM duty (0-1023)
            duty = int(power / self.config.max_current * 1023)
            self.motor_pwm.duty(duty)
        elif self.config.motor_controller_type == CTRL_UART:
            # Send UART command (protocol specific)
            cmd = bytearray(4)
            cmd[0] = 0x01  # Set power command
            cmd[1] = int(power * 10)  # Send as 0.1A units
            self.motor_uart.write(cmd)
        elif self.config.motor_controller_type == CTRL_CAN:
            # Send CAN message
            msg = bytearray(8)
            msg[0] = 0x01  # Power control command
            msg[1] = int(power)  # Current in Amps
            self.motor_can.send(msg, 0x123)  # Example CAN ID
        elif self.config.motor_controller_type == CTRL_VESC:
            # Send VESC command
            current = int(power * 1000)  # mA
            cmd = bytearray(5)
            cmd[0] = 0x04  # Set current command
            cmd[1:4] = ustruct.pack('<i', current)  # Little-endian 32-bit
            self._send_vesc_command(cmd)
    
    def _update_display(self):
        """Update display based on display type"""
        if self.display is None:
            return
            
        if self.config.display_type == DISPLAY_OLED_I2C:
            # OLED I2C display
            self.display.fill(0)
            
            # Speed and battery
            self.display.text("Speed: {:.1f}km/h".format(self.speed), 0, 0)
            self.display.text("Batt: {}%".format(self.battery_level), 0, 10)
            
            # Torque if available
            if self.config.has_torque_sensor:
                self.display.text("Torque: {:.1f}Nm".format(self.torque), 0, 20)
            
            # Assist level
            self.display.text("Mode: {}".format(
                "Eco" if self.assist_level == 1 else (
                "Normal" if self.assist_level == 2 else "Sport")), 0, 30)
            
            # Power and regen status
            status = "Power: {}W".format(int(self.motor_power))
            if self.regenerative_braking:
                status += " REGEN"
            self.display.text(status, 0, 40)
            
            # Lights and charging status
            status = "Lights: {}".format("ON" if self.lights_on else "OFF")
            if self.charging:
                status += " CHG"
            self.display.text(status, 0, 50)
            
            self.display.show()
            
        elif self.config.display_type == DISPLAY_LCD_SPI:
            # SPI LCD display - implement your specific LCD commands
            pass
            
        elif self.config.display_type == DISPLAY_TFT:
            # TFT display with more advanced graphics
            pass
    
    def _check_safety(self):
        """Check all safety limits and take appropriate action"""
        # Over temperature protection
        if self.motor_temp > self.max_motor_temp or self.controller_temp > self.max_controller_temp:
            self._set_motor_power(0)
            if self.display is not None:
                self.display.text("OVER TEMP!", 0, 50)
                self.display.show()
        
        # Low battery warning
        if self.battery_level < self.low_battery_threshold:
            if self.display is not None:
                self.display.text("LOW BATTERY", 0, 50)
                self.display.show()
    
    def toggle_lights(self):
        self.lights_on = not self.lights_on
        self.headlight.value(self.lights_on)
        self.taillight.value(self.lights_on)
    
    def sound_horn(self):
        self.horn.value(1)
        self.horn_on = True
        # Start timer to turn off horn after 1 second
        Timer(-1).init(period=1000, mode=Timer.ONE_SHOT, callback=lambda t: self._stop_horn())
    
    def _stop_horn(self):
        self.horn.value(0)
        self.horn_on = False
    
    def cycle_assist_level(self):
        self.assist_level = self.assist_level % 3 + 1

# Main loop
if __name__ == "__main__":
    ebike = EBikeSystem()
    
    try:
        while True:
            # Main loop can be used for non-time-critical tasks
            time.sleep(1)
    except KeyboardInterrupt:
        ebike._set_motor_power(0)
        if ebike.display is not None:
            ebike.display.fill(0)
            ebike.display.text("System OFF", 0, 0)
            ebike.display.show()