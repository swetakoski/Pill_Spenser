import time
import machine
import os
import json

# Constants
PILL_COMPARTMENTS = 7  # Number of compartments with pills
MOTOR_PIN = machine.Pin(15, machine.Pin.OUT)  # Motor control pin
BUTTON_PIN = machine.Pin(14, machine.Pin.IN, machine.Pin.PULL_UP)  # Button pin
LED_PIN = machine.Pin(13, machine.Pin.OUT)  # LED pin
SENSOR_PIN = machine.Pin(12, machine.Pin.IN)  # Piezoelectric sensor pin
DELAY_BETWEEN_DISPENSES = 30  # 30 seconds for testing (can be changed later)
LOREN_MSG_PORT = 8  # LoRaWAN message port
DEVICE_ID = "unique_device_id"  # Placeholder for device identifier

# Non-volatile memory (e.g., for storing the state)
STATE_FILE = '/state.json'

# LoRaWAN communication module mock-up for simulation
class LoRaWAN:
    def send_command(self, command):
        print(f"Sending command: {command}")
        time.sleep(1)
    
    def receive_message(self):
        return "Received message"

# Instantiate LoRaWAN object
lora = LoRaWAN()

# Initialize LoRaWAN
def init_lora():
    lora.send_command("+MODE=LWOTAA")
    time.sleep(1)
    lora.send_command("+KEY=APPKEY,\"<your_app_key_here>\"")
    time.sleep(1)
    lora.send_command("+CLASS=A")
    time.sleep(1)
    lora.send_command("+PORT=8")
    time.sleep(1)
    lora.send_command("+JOIN")
    # Wait for join to complete
    time.sleep(20)

# Load or initialize state
def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, 'r') as f:
            state = json.load(f)
    else:
        state = {
            "pills_left": PILL_COMPARTMENTS * 10,  # Example: 10 pills per compartment
            "dispensed_today": 0,
            "last_calibration_time": 0
        }
    return state

def save_state(state):
    with open(STATE_FILE, 'w') as f:
        json.dump(state, f)

# Button press detection
def wait_for_button():
    LED_PIN.on()  # Blink LED while waiting for press
    while BUTTON_PIN.value() == 1:
        time.sleep(0.1)
    LED_PIN.off()  # Button pressed, LED goes off

# Perform calibration (rotate wheel and align sensor)
def calibrate():
    LED_PIN.on()  # Indicate calibration in progress
    MOTOR_PIN.on()  # Turn motor
    time.sleep(5)  # Simulate one full rotation for calibration
    MOTOR_PIN.off()  # Stop motor
    # Here you would rotate it based on the actual sensor
    time.sleep(1)  # Pause for alignment
    LED_PIN.off()  # Calibration done

# Dispense pill
def dispense_pill(state):
    MOTOR_PIN.on()
    time.sleep(1)  # Simulate wheel turning
    MOTOR_PIN.off()

    # Check if pill was dispensed (using piezoelectric sensor)
    if SENSOR_PIN.value() == 0:  # No pill detected
        LED_PIN.blink(500, 5)  # Blink LED 5 times
    else:
        state["dispensed_today"] += 1  # Increment dispensed pills count
        state["pills_left"] -= 1  # Decrease number of pills
        save_state(state)  # Save updated state

def send_lora_message(message):
    lora.send_command(f"+MSG={message}")
    time.sleep(5)

def main():
    # Initialize device state
    state = load_state()

    # Initialize LoRaWAN
    init_lora()

    # Main loop
    while True:
        wait_for_button()  # Wait for user to press button
        calibrate()  # Perform calibration

        # Loop to dispense pills
        while state["pills_left"] > 0 and state["dispensed_today"] < PILL_COMPARTMENTS:
            dispense_pill(state)
            time.sleep(DELAY_BETWEEN_DISPENSES)

        if state["pills_left"] == 0:
            send_lora_message(f"{DEVICE_ID} - Dispenser Empty")
            state["pills_left"] = PILL_COMPARTMENTS * 10  # Reload pills for testing
            save_state(state)

        # Device goes back to waiting for calibration
        wait_for_button()

if __name__ == "__main__":
    main()
