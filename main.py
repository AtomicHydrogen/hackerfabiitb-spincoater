import rp2
from machine import Pin, UART
import time
import _thread
import sys

# -----------------------------------------------------------------------------
# 1. GLOBAL VARIABLES & SHARED STATE
# -----------------------------------------------------------------------------
measured_rpm = 0
rpm_lock = _thread.allocate_lock()

# --- Shared state for motor control ---
class MotorState:
    IDLE = 0     # Not armed
    ARMED = 1    # Armed, motor stopped
    STARTING = 2 # Fixed 4-second low-throttle startup
    RUNNING = 3  # PID controller is active during setup time
    HOLDING = 4  # PID remains active to prevent drift during hold time

motor_state = MotorState.IDLE
target_rpm = 0

# -----------------------------------------------------------------------------
# 2. PID CONTROLLER CLASS (NEW)
# -----------------------------------------------------------------------------
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, motor_constant, output_limits):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.motor_constant = motor_constant
        self.min_out, self.max_out = output_limits
        
        # Internal variables
        self._integral = 0
        self._last_error = 0
        self._last_time = time.ticks_ms()

    def compute(self, current_value):
        now = time.ticks_ms()
        dt_ms = time.ticks_diff(now, self._last_time)
        
        if dt_ms == 0: return None
        
        dt = dt_ms / 1000.0
        error = self.setpoint - current_value
        
        p_term = self.Kp * (error / self.motor_constant) * 2047
        
        self._integral += error * dt
        i_term = self.Ki * (self._integral / self.motor_constant) * 2047
        
        derivative = (error - self._last_error) / dt
        d_term = self.Kd * (derivative / self.motor_constant) * 2047
        
        output = (self.setpoint / self.motor_constant) * 2047
        output += p_term + i_term + d_term
        
        output = max(min(output, self.max_out), self.min_out)
        if output == self.max_out or output == self.min_out:
             self._integral -= error * dt # Anti-windup
        
        self._last_error = error
        self._last_time = now
        
        return int(output)

    def set_target(self, setpoint):
        self.setpoint = setpoint
        self.reset()

    def reset(self):
        self._integral = 0
        self._last_error = 0
        self._last_time = time.ticks_ms()
        
# -----------------------------------------------------------------------------
# 3. RPM COUNTER THREAD (RUNS ON CORE 1) - Unchanged
# -----------------------------------------------------------------------------
def rpm_counter_thread():
    import time
    global measured_rpm
    hall_sensor_pins = [Pin(2, Pin.IN, Pin.PULL_UP), Pin(3, Pin.IN, Pin.PULL_UP)]
    last_states = [p.value() for p in hall_sensor_pins]
    pulse_counter = 0; last_update_time = time.ticks_ms()
    while True:
        for i, pin in enumerate(hall_sensor_pins):
            current_state = pin.value()
            if current_state == 1 and last_states[i] == 0: pulse_counter += 1
            last_states[i] = current_state
        if time.ticks_diff(time.ticks_ms(), last_update_time) >= 1000:
            rpm_lock.acquire()
            measured_rpm = pulse_counter * (60 / len(hall_sensor_pins))
            rpm_lock.release()
            pulse_counter = 0; last_update_time = time.ticks_ms()

# -----------------------------------------------------------------------------
# 4. PIO & DSHOT - Unchanged
# -----------------------------------------------------------------------------
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def dshot_encoder_pio():
    ONE_HIGH_DELAY=29;ONE_LOW_DELAY=5;ZERO_HIGH_DELAY=14;ZERO_LOW_DELAY=20;FRAME_DELAY=29;FRAME_DELAY_COUNT=26;FRAME_DELAY_REMAINDER=19;label("init");jmp("blocking_pull");label("maybe_pull");mov(y,isr);jmp(not_y,"blocking_pull");pull(noblock);nop()[FRAME_DELAY_REMAINDER];set(y,FRAME_DELAY_COUNT);label("frame_delay_loop");jmp(not_y,"start_frame")[FRAME_DELAY];jmp(y_dec,"frame_delay_loop");label("blocking_pull");pull(block);label("start_frame");mov(x,osr);jmp(not_x,"blocking_pull");out(y,16);label("check_bit");jmp(not_osre,"start_bit");jmp("maybe_pull");label("start_bit");out(y,1);jmp(not_y,"do_zero");label("do_one");set(pins,1)[ONE_HIGH_DELAY];set(pins,0)[ONE_LOW_DELAY];jmp("check_bit");label("do_zero");set(pins,1)[ZERO_HIGH_DELAY];set(pins,0)[ZERO_LOW_DELAY];jmp("check_bit")

class DShot:
    BIT_PERIOD = 40
    def __init__(self, pin_num, speed_khz, frame_rate_hz=2000, enable_repeat=True):
        self.pin = Pin(pin_num);self.speed_khz = speed_khz;self.enable_repeat = enable_repeat
        pio_freq = self.BIT_PERIOD * self.speed_khz * 1000
        if self.enable_repeat:
            frame_time_us = 16 * (1000 / self.speed_khz);target_period_us = 1_000_000 / frame_rate_hz
            delay_us = target_period_us - frame_time_us
            if delay_us < 0: self.delay_cycles = 0
            else: self.delay_cycles = int(delay_us * (pio_freq / 1_000_000)) - 7
        self.sm = rp2.StateMachine(0, dshot_encoder_pio, freq=pio_freq, set_base=self.pin)
        self.sm.put(1 if self.enable_repeat else 0); self.sm.exec("pull()")
        self.sm.exec("mov(isr, osr)"); self.sm.active(1)
    def _create_packet(self, throttle, telemetry):
        packet = (throttle << 1) | (1 if telemetry else 0); crc = 0; c = packet
        for _ in range(3): crc ^= c; c >>= 4
        return (packet << 4) | (crc & 0x0F)
    def send_throttle(self, throttle, telemetry=False):
        if not (0 <= throttle <= 2047): raise ValueError("Throttle must be 0-2047.")
        packet = self._create_packet(throttle, telemetry)
        if self.enable_repeat:
            self.sm.put(self.delay_cycles // 32); self.sm.put(packet)
        else: self.sm.put(packet)

# -----------------------------------------------------------------------------
# 5. MAIN EXECUTION BLOCK (RUNS ON CORE 0) - Reworked with uart.write()
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    # --- Configuration ---
    DSHOT_PIN = 0; DSHOT_SPEED = 150
    MIN_THROTTLE = 100; MAX_THROTTLE = 2047
    MOTOR_CONSTANT = 935.0 * 12.0
    KP = 0.003; KI = 0.5; KD = 0.01
    
    # --- Setup ---
    uart = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13)); uart_buffer = ""
    esc = DShot(DSHOT_PIN, DSHOT_SPEED, frame_rate_hz=500, enable_repeat=True)
    _thread.start_new_thread(rpm_counter_thread, ())
    pid = PID(KP, KI, KD, setpoint=0, motor_constant=MOTOR_CONSTANT, output_limits=(MIN_THROTTLE, MAX_THROTTLE))

    # --- State machine variables ---
    hold_time_ms = 0; setup_time_ms = 0; state_timer_start = 0; current_throttle = 0

    try:
        while True:
            # --- 1. Check for UART commands ---
            if uart.any():
                uart_buffer += uart.read().decode('utf-8')
                if '\n' in uart_buffer:
                    command_line, uart_buffer = uart_buffer.split('\n', 1)
                    command_line = command_line.strip()
                    if command_line == "arm":
                        if motor_state == MotorState.IDLE:
                            uart.write("UART: Arming...\n")
                            motor_state = MotorState.ARMED
                            for _ in range(50): esc.send_throttle(0); time.sleep_ms(10)
                            uart.write("✅ ESC Armed. Ready for ramp command.\n")
                        else: uart.write("Warn: Already armed.\n")
                    elif command_line.startswith("ramp(") and command_line.endswith(")"):
                        if motor_state == MotorState.ARMED:
                            try:
                                params_str = command_line[5:-1]; params = params_str.split(',')
                                target_rpm = int(params[0].strip())
                                setup_time_ms = int(params[1].strip())
                                hold_time_ms = int(params[2].strip())
                                uart.write(f"UART: Ramping to {target_rpm} RPM...\n")
                                motor_state = MotorState.STARTING; state_timer_start = time.ticks_ms()
                            except (ValueError, IndexError): uart.write("Error: Invalid ramp parameters.\n")
                        else: uart.write("Error: Not armed. Send 'arm' command first.\n")
                    elif command_line == "interrupt":
                        uart.write("UART: Interrupt received. Stopping motor.\n")
                        target_rpm = 0; current_throttle = 0
                        motor_state = MotorState.ARMED; esc.send_throttle(0)
                        
            # --- 2. Main Motor Control State Machine ---
            if motor_state == MotorState.STARTING:
                esc.send_throttle(100)
                if time.ticks_diff(time.ticks_ms(), state_timer_start) >= 4000:
                    uart.write("Startup complete. Engaging PID.\n")
                    motor_state = MotorState.RUNNING
                    state_timer_start = time.ticks_ms(); pid.set_target(target_rpm)

            elif motor_state == MotorState.RUNNING:
                rpm_lock.acquire(); current_rpm = measured_rpm; rpm_lock.release()
                throttle_value = pid.compute(current_rpm)
                if throttle_value is not None:
                    current_throttle = throttle_value; esc.send_throttle(current_throttle)
                
                if time.ticks_diff(time.ticks_ms(), state_timer_start) >= setup_time_ms:
                    uart.write("Setup time finished. Holding RPM with active PID.\n")
                    motor_state = MotorState.HOLDING; state_timer_start = time.ticks_ms()
                    
            elif motor_state == MotorState.HOLDING:
                rpm_lock.acquire(); current_rpm = measured_rpm; rpm_lock.release()
                throttle_value = pid.compute(current_rpm)
                if throttle_value is not None:
                    current_throttle = throttle_value; esc.send_throttle(current_throttle)

                if time.ticks_diff(time.ticks_ms(), state_timer_start) >= hold_time_ms:
                    uart.write("Hold time finished. Returning to ARMED state.\n")
                    esc.send_throttle(0); current_throttle = 0; motor_state = MotorState.ARMED
            
            time.sleep_ms(20) # Main loop delay

    except KeyboardInterrupt:
        uart.write("Keyboard interrupt. Halting.\n")
    finally:
        for _ in range(50): esc.send_throttle(0); time.sleep_ms(10)

