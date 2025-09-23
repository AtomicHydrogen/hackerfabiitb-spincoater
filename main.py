import rp2
from machine import Pin
import time

# -----------------------------------------------------------------------------
# 1. THE PIO PROGRAM FOR DSHOT
# -----------------------------------------------------------------------------
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def dshot_encoder_pio():
    # --- Define Constants (from original .pio file) ---
    # Total bit period is 40 PIO cycles.
    # Delay values are -1 because MicroPython assembler includes the instruction cycle.
    ONE_HIGH_DELAY = 29  # (30 - 1)
    ONE_LOW_DELAY = 5    # (10 - 5 instructions)
    ZERO_HIGH_DELAY = 14 # (15 - 1)
    ZERO_LOW_DELAY = 20  # (25 - 5 instructions)
    
    # Fixed frame delay constants from the original C++ PIO file.
    FRAME_DELAY = 29           # (32 - 2 - 1)
    FRAME_DELAY_COUNT = 26
    FRAME_DELAY_REMAINDER = 19 # (27 - 7 - 1)

    # --- Start of program ---
    label("init")
    # This jump is present in the original .pio file.
    # The first two instructions (pull, mov) are executed manually by the Python class.
    jmp("blocking_pull")

    label("maybe_pull")
    mov(y, isr)
    jmp(not_y, "blocking_pull")
    pull(noblock)
    # Repeat mode is enabled, delay is needed to control frame rate
    nop() [FRAME_DELAY_REMAINDER]
    set(y, FRAME_DELAY_COUNT)
    
    label("frame_delay_loop")
    jmp(not_y, "start_frame") [FRAME_DELAY]
    jmp(y_dec, "frame_delay_loop")

    label("blocking_pull")
    pull(block)
    
    label("start_frame")
    mov(x, osr)              # Store the value
    jmp(not_x, "blocking_pull") # wait for non-zero value
    out(y, 16)               # discard 16 most significant bits

    label("check_bit")
    jmp(not_osre, "start_bit")
    jmp("maybe_pull")

    label("start_bit")
    out(y, 1)     # Pull one bit into Y
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
# 2. THE PYTHON CLASS TO CONTROL THE PIO
# -----------------------------------------------------------------------------
class DShot:
    BIT_PERIOD = 40
    
    def __init__(self, pin, speed_khz, frame_rate_hz=2000, enable_repeat=True):
        self.pin = pin
        self.speed_khz = speed_khz
        self.enable_repeat = enable_repeat

        pio_freq = self.BIT_PERIOD * self.speed_khz * 1000
        
        if self.enable_repeat:
            frame_time_us = 16 * (1000 / self.speed_khz)
            target_period_us = 1_000_000 / frame_rate_hz
            delay_us = target_period_us - frame_time_us
            if delay_us < 0:
                self.delay_cycles = 0
            else:
                pio_cycles_per_us = pio_freq / 1_000_000
                self.delay_cycles = int(delay_us * pio_cycles_per_us) - 7
        
        self.sm = rp2.StateMachine(0, dshot_encoder_pio, freq=pio_freq, set_base=Pin(0))

        self.sm.put(1 if self.enable_repeat else 0)
        self.sm.exec("pull()")
        self.sm.exec("mov(isr, osr)")
        self.sm.active(1)

    def _create_packet(self, throttle, telemetry):
        packet = (throttle << 1) | (1 if telemetry else 0)
        crc = 0
        c = packet
        for _ in range(3):
            crc ^= c
            c >>= 4
        crc &= 0x0F
        return (packet << 4) | crc

    def send_throttle(self, throttle, telemetry=False):
        if not (0 <= throttle <= 2047):
            raise ValueError("Throttle must be between 0 and 2047.")
            
        packet = self._create_packet(throttle, telemetry)
        
        if self.enable_repeat:
            delay_loop_count = self.delay_cycles // 32
            self.sm.put(delay_loop_count)
            self.sm.put(packet)
        else:
            self.sm.put(packet)

# -----------------------------------------------------------------------------
# 3. EXAMPLE USAGE (THIS PART RUNS WHEN YOU CLICK "RUN" IN THONNY)
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    # --- Configuration ---
    # CHANGE THESE VALUES TO MATCH YOUR SETUP
    DSHOT_PIN = 0         # The GPIO pin connected to your ESC's signal wire
    DSHOT_SPEED = 150     # DShot speed in kHz (150, 300, or 600)
    FRAME_RATE = 10     # How many times per second to send a command (Hz)

    # --- Initialization ---
    esc = DShot(DSHOT_PIN, DSHOT_SPEED, frame_rate_hz=FRAME_RATE, enable_repeat=True)

    # --- Arming Sequence ---
    # ⚠️ WARNING: REMOVE PROPELLER BEFORE RUNNING
    print("--- DShot Motor Demo ---")
    print("⚠️ REMOVE PROPELLER BEFORE CONTINUING! ⚠️")
    print("Arming ESC in 3 seconds...")
    time.sleep(3)
    
    # Send 0 throttle for a short period to arm the ESC
    for _ in range(50):
        esc.send_throttle(0)
        time.sleep_ms(10)
    print("✅ ESC Armed.")
    time.sleep(1)

    # --- Motor Control Loop ---
    try:
        while True:
            # Ramp up from 5% to 25% throttle
            print("Ramping up...")
            for throttle in range(100, 2000, 50): # Min throttle is 48, max is 2047
                esc.send_throttle(throttle)
                time.sleep_ms(500)
            
            time.sleep(1)

            # Ramp down
            print("Ramping down...")
            for throttle in range(2000, 99, -50):
                esc.send_throttle(throttle)
                time.sleep_ms(500)

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping motor.")
        # Always send 0 throttle on exit to safely stop the motor
        for _ in range(20):
            esc.send_throttle(0)
            time.sleep_ms(1)