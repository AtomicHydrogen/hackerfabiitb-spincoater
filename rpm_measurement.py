import rp2
from machine import Pin
import time
import _thread

# -----------------------------------------------------------------------------
# 1. GLOBAL VARIABLES & THREAD LOCK
# -----------------------------------------------------------------------------
# This variable will be shared between the two threads.
# The 'global' keyword will be used to access it from functions.
measured_rpm = 0
rpm_lock = _thread.allocate_lock()


# -----------------------------------------------------------------------------
# 2. RPM COUNTER THREAD (RUNS ON CORE 1)
# -----------------------------------------------------------------------------
def rpm_counter_thread():
    """
    This function runs on the Pico's second core. Its only job is to
    poll GPIO pins GP2-GP5 as fast as possible to count motor revolutions.
    """
    global measured_rpm # Use the global variable
    
    # Configure a list of input pins for the Hall effect sensors
    hall_sensor_pins = [
        Pin(2, Pin.IN, Pin.PULL_UP)
        #Pin(3, Pin.IN, Pin.PULL_UP),
        #Pin(4, Pin.IN, Pin.PULL_UP),
        #Pin(5, Pin.IN, Pin.PULL_UP)
    ]
    
    # Create a list to store the last state of each pin individually
    last_states = [p.value() for p in hall_sensor_pins]
    
    pulse_counter = 0
    last_update_time = time.ticks_ms()

    print("✅ RPM counter thread started on Core 1 (polling GP2-GP5).")

    # This is the high-speed polling loop. It never sleeps.
    while True:
        # Iterate through each pin to check its state
        for i, pin in enumerate(hall_sensor_pins):
            current_state = pin.value()

            # Detect a rising edge (from LOW to HIGH) for the current pin
            if current_state == 1 and last_states[i] == 0:
                pulse_counter += 1
            
            # Update the last state for the current pin
            last_states[i] = current_state

        # Check if 1 second has passed to update the RPM (this happens after checking all pins)
        if (time.ticks_ms() - last_update_time) >= 1000:
            # --- Critical Section: Safely update the global RPM ---
            rpm_lock.acquire()
            # Formula: RPM = pulses_per_second * 60 / pulses_per_revolution
            # Your formula (RPM = 15 * ctr) implies 4 pulses per revolution (60/4=15)
            measured_rpm = pulse_counter * (60/len(hall_sensor_pins))
            rpm_lock.release()
            # ---------------------------------------------------------
            
            # Reset the counter and the timer for the next 1-second interval
            pulse_counter = 0
            last_update_time = time.ticks_ms()

# -----------------------------------------------------------------------------
# 3. PIO PROGRAM FOR DSHOT (Unchanged)
# -----------------------------------------------------------------------------
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def dshot_encoder_pio():
    # --- Define Constants (from original .pio file) ---
    ONE_HIGH_DELAY = 29
    ONE_LOW_DELAY = 5
    ZERO_HIGH_DELAY = 14
    ZERO_LOW_DELAY = 20
    FRAME_DELAY = 29
    FRAME_DELAY_COUNT = 26
    FRAME_DELAY_REMAINDER = 19

    # --- Start of program ---
    label("init")
    jmp("blocking_pull")
    label("maybe_pull")
    mov(y, isr)
    jmp(not_y, "blocking_pull")
    pull(noblock)
    nop() [FRAME_DELAY_REMAINDER]
    set(y, FRAME_DELAY_COUNT)
    label("frame_delay_loop")
    jmp(not_y, "start_frame") [FRAME_DELAY]
    jmp(y_dec, "frame_delay_loop")
    label("blocking_pull")
    pull(block)
    label("start_frame")
    mov(x, osr)
    jmp(not_x, "blocking_pull")
    out(y, 16)
    label("check_bit")
    jmp(not_osre, "start_bit")
    jmp("maybe_pull")
    label("start_bit")
    out(y, 1)
    jmp(not_y, "do_zero")
    label("do_one")
    set(pins, 1) [ONE_HIGH_DELAY]
    set(pins, 0) [ONE_LOW_DELAY]
    jmp("check_bit")
    label("do_zero")
    set(pins, 1) [ZERO_HIGH_DELAY]
    set(pins, 0) [ZERO_LOW_DELAY]
    jmp("check_bit")

# -----------------------------------------------------------------------------
# 4. DSHOT CLASS (Unchanged)
# -----------------------------------------------------------------------------
class DShot:
    BIT_PERIOD = 40
    
    def __init__(self, pin_num, speed_khz, frame_rate_hz=2000, enable_repeat=True):
        self.pin = Pin(pin_num)
        self.speed_khz = speed_khz
        self.enable_repeat = enable_repeat
        pio_freq = self.BIT_PERIOD * self.speed_khz * 1000
        if self.enable_repeat:
            frame_time_us = 16 * (1000 / self.speed_khz)
            target_period_us = 1_000_000 / frame_rate_hz
            delay_us = target_period_us - frame_time_us
            if delay_us < 0: self.delay_cycles = 0
            else: self.delay_cycles = int(delay_us * (pio_freq / 1_000_000)) - 7
        self.sm = rp2.StateMachine(0, dshot_encoder_pio, freq=pio_freq, set_base=self.pin)
        self.sm.put(1 if self.enable_repeat else 0)
        self.sm.exec("pull()")
        self.sm.exec("mov(isr, osr)")
        self.sm.active(1)

    def _create_packet(self, throttle, telemetry):
        packet = (throttle << 1) | (1 if telemetry else 0)
        crc = 0; c = packet
        for _ in range(3): crc ^= c; c >>= 4
        return (packet << 4) | (crc & 0x0F)

    def send_throttle(self, throttle, telemetry=False):
        if not (0 <= throttle <= 2047): raise ValueError("Throttle must be between 0 and 2047.")
        packet = self._create_packet(throttle, telemetry)
        if self.enable_repeat:
            self.sm.put(self.delay_cycles // 32)
            self.sm.put(packet)
        else: self.sm.put(packet)

# -----------------------------------------------------------------------------
# 5. MAIN EXECUTION BLOCK (RUNS ON CORE 0)
# -----------------------------------------------------------------------------

def handle_uart_commands():
    # This is where you will add your UART command processing logic.
    # For example, it could read from sys.stdin and set a 'target_rpm' variable.
    pass

if __name__ == "__main__":
    # --- Configuration ---
    DSHOT_PIN = 0
    DSHOT_SPEED = 150
    
    # --- Initialization ---
    esc = DShot(DSHOT_PIN, DSHOT_SPEED, frame_rate_hz=100, enable_repeat=True)
    
    # --- Start the second thread ---
    _thread.start_new_thread(rpm_counter_thread, ())

    # --- Data Logging Setup ---
    LOG_FILE_NAME = "motor_log.csv"
    try:
        logfile = open(LOG_FILE_NAME, "w")
        # Write the header for the CSV file
        logfile.write("timestamp_ms,target_rpm,measured_rpm,throttle_sent\n")
        print(f"✅ Logging data to {LOG_FILE_NAME}")
    except OSError as e:
        print(f"❌ Error opening log file: {e}")
        logfile = None # Set to None so we don't try to write to it


    # --- Arming Sequence ---
    print("--- DShot Motor Control with RPM Feedback ---")
    print("⚠️ REMOVE PROPELLER BEFORE CONTINUING! ⚠️")
    print("Arming ESC in 3 seconds...")
    time.sleep(3)
    for _ in range(50):
        esc.send_throttle(0)
        time.sleep_ms(10)
    print("✅ ESC Armed.")
    time.sleep(1)

    # --- Main Motor Control Loop ---
    target_rpm = 0 # This would be set by your UART commands
    start_time = time.ticks_ms()
    
    # Initialize variables for throttle ramping logic
    temp_ctr = 1
    temp_flag = 0

    try:
        while True:
            # 1. Handle incoming commands (e.g., set a new target_rpm)
            handle_uart_commands()
            
            # 2. Read the latest RPM value safely from the global variable
            rpm_lock.acquire()
            current_rpm = measured_rpm
            rpm_lock.release()

            # 3. Print status for debugging
            # We use \r (carriage return) to print on the same line
            print(f"Target RPM: {target_rpm} | Current RPM: {current_rpm}    ", end='\r')
            
            # 4. Implement the throttle ramping logic
            throttle_value = 100 * temp_ctr
            if temp_flag == 0:
                temp_ctr += 1
            else:
                temp_ctr -= 1
            
            if temp_ctr >= 20: # Use >= to ensure it triggers
                temp_flag = 1
            elif temp_ctr <= 1: # Use <= to ensure it triggers
                temp_flag = 0

            esc.send_throttle(throttle_value)

            # 5. Log the data to the file
            if logfile:
                print("Creating log file")
                timestamp = time.ticks_diff(time.ticks_ms(), start_time)
                print("diff worked?")
                # We can log the ramping throttle as the "target" for now
                log_line = f"{timestamp},{throttle_value},{current_rpm},{throttle_value}\n"
                logfile.write(log_line)
            
            # 6. Set the update rate of the main control loop.
            time.sleep_ms(50)

    except KeyboardInterrupt:
        print("\nStopping motor.")
        
        # Close the log file safely before stopping the script
        if logfile:
            logfile.close()
            print(f"✅ Log file '{LOG_FILE_NAME}' closed.")

        # Gracefully stop the motor on exit
        for _ in range(50):
            esc.send_throttle(0)
            time.sleep_ms(10)


