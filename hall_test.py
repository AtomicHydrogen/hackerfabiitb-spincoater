import machine
import time

# Configure GP0 as an input pin with an internal pull-down resistor.
# The pull-down ensures the pin reads LOW (0) when nothing is connected or the switch is open.
pin = machine.Pin(2, machine.Pin.IN, machine.Pin.PULL_UP)

# This variable will store the previous state of the pin to help us detect a change.
last_state = pin.value()
ctr = 0

print("--- Starting GPIO Polling ---")
print("Monitoring GP0. Connect GP0 to 3.3V to trigger.")

# An infinite loop to continuously check the pin's state.
while True:
    current_state = pin.value()

    # Check for a rising edge: The pin just transitioned from LOW to HIGH.
    if current_state == 1 and last_state == 0:
        print(">> FLAG: Rising Edge Detected (x",ctr,")")
        ctr += 1

    # Check if the pin is currently HIGH.
    # This will be true on the rising edge and for as long as the pin is held high.
    if current_state == 1:
        print("   - FLAG: Pin is HIGH.")

    # Update the last state for the next iteration of the loop.
    last_state = current_state

    # Wait for a short period before polling again to avoid flooding the console.
    time.sleep_ms(100) # Check 10 times per second.