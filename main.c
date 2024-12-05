#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// GPIO Pin Assignments
#define SW_0 9            // Dispense button GPIO pin
#define SW_2 7            // Calibration button GPIO pin
#define LED_1 21          // LED GPIO pin
#define IN1 2             // Stepper motor GPIO pin 1
#define IN2 3             // Stepper motor GPIO pin 2
#define IN3 6             // Stepper motor GPIO pin 3
#define IN4 13            // Stepper motor GPIO pin 4
#define OPTO_FORK 28      // Optofork sensor GPIO pin
#define PIEZO_SENSOR 27   // Piezoelectric sensor GPIO pin

// Timing Constants
#define STEP_DELAY_MS 2      // Step delay for the stepper motor
#define DEBOUNCE_DELAY_US 200000 // Debounce delay for button presses
#define ERROR_BLINKS 5       // Number of LED blinks for errors
#define ERROR_BLINK_DELAY 200 // LED blink duration for errors

// Stepper Motor Control Sequence

const int half_step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

int steps_per_revolution = 0;

// Enum for Dispenser States
typedef enum {
    STATE_WAIT,
    STATE_CALIBRATION,
    STATE_DISPENSE_PILLS,
} dispenser_state;

// Global Variables
bool sensor_triggered = false;
bool system_calibrated = false;

// Function Prototypes
void initialize_gpio();
void activate_motor_step();
void calibrate_motor();
void run_motor(int steps);
void idle_mode();
void calibrating();
void dispensing_pills();
void handle_error();
void piezo_interrupt(uint gpio, uint32_t events);

// Interrupt Callback for Piezo Sensor
void piezo_interrupt(uint gpio, uint32_t events) {
    if (gpio == PIEZO_SENSOR && events & GPIO_IRQ_EDGE_FALL) {
        sensor_triggered = true;
    }
}

int main() {
    stdio_init_all();
    initialize_gpio();

    dispenser_state current_state = STATE_WAIT;
    printf("System started. Entering idle mode.Press SW_2 to calibrate...\n");

    while (1) {
        switch (current_state) {
            case STATE_WAIT:
                idle_mode();
                if (gpio_get(SW_2) == 0 && !system_calibrated) {
                    printf("Calibration button pressed. Starting calibration...\n");
                    current_state = STATE_CALIBRATION;
                }
                break;

            case STATE_CALIBRATION:
                calibrating();
                system_calibrated = true;
                current_state = STATE_DISPENSE_PILLS;
                break;

            case STATE_DISPENSE_PILLS:
                if (gpio_get(SW_0) == 0) {
                    printf("Dispense button pressed. Dispensing pills...\n");
                    dispensing_pills();
                }
                current_state = STATE_DISPENSE_PILLS;
                break;
        }
    }
}

// GPIO Initialization
void initialize_gpio() {
    gpio_init(SW_0);
    gpio_set_dir(SW_0, GPIO_IN);
    gpio_pull_up(SW_0);

    gpio_init(SW_2);
    gpio_set_dir(SW_2, GPIO_IN);
    gpio_pull_up(SW_2);

    gpio_init(LED_1);
    gpio_set_dir(LED_1, GPIO_OUT);
    gpio_put(LED_1, false);

    gpio_init(IN1);
    gpio_init(IN2);
    gpio_init(IN3);
    gpio_init(IN4);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_set_dir(IN4, GPIO_OUT);

    gpio_init(OPTO_FORK);
    gpio_set_dir(OPTO_FORK, GPIO_IN);
    gpio_pull_up(OPTO_FORK);

    gpio_init(PIEZO_SENSOR);
    gpio_set_dir(PIEZO_SENSOR, GPIO_IN);
    gpio_pull_up(PIEZO_SENSOR);

    gpio_set_irq_enabled_with_callback(PIEZO_SENSOR, GPIO_IRQ_EDGE_FALL, true, piezo_interrupt);
}

// Activate Motor Step
void activate_motor_step() {
    static int step_index = 0;
    step_index = (step_index + 1) % 8;
    gpio_put(IN1, half_step_sequence[step_index][0]);
    gpio_put(IN2, half_step_sequence[step_index][1]);
    gpio_put(IN3, half_step_sequence[step_index][2]);
    gpio_put(IN4, half_step_sequence[step_index][3]);
    sleep_ms(STEP_DELAY_MS);
}

// Calibrate Motor
void calibrate_motor() {
    printf("Calibrating motor...\n");
    int step_counts[3];
    for (int i = 0; i < 2; i++) {
        int count = 0;
        while (gpio_get(OPTO_FORK)) {  // find the first falling edge
            activate_motor_step();
            sleep_ms(STEP_DELAY_MS);
            count++;
        }
        count = 0;
        while (!gpio_get(OPTO_FORK)) {
            activate_motor_step();
            sleep_ms(STEP_DELAY_MS);
            count++;
        }
        // measure steps for one full revolution
        while (gpio_get(OPTO_FORK)) {
            activate_motor_step();
            sleep_ms(STEP_DELAY_MS);
            count++;
        }

        step_counts[i] = count;

    }
    steps_per_revolution = (step_counts[0] + step_counts[1]  / 2);
    for (int i = 0; i < 142; i++) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
    }
    printf("Machine has been calibrated. Press SW_0 for pill dispenser....\n");
}


// Run Motor
void run_motor(int N) {
    for (int i = 0; i < 512; i++) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
    }
}

// Idle Mode
void idle_mode() {
    gpio_put(LED_1, true);
    sleep_ms(200);
    gpio_put(LED_1, false);
    sleep_ms(200);
}

// Calibrating
void calibrating() {
    gpio_put(LED_1, true);
    calibrate_motor();
    gpio_put(LED_1, false);
}

// Dispensing Pills
void dispensing_pills() {
    gpio_put(LED_1, 0);
    int step_and_stop = 0;
    int steps_per_compartment = steps_per_revolution / 7; // Calculate steps per compartment

    while(step_and_stop < 7){
        sensor_triggered=false;
        // Rotate motor to the next compartment
        run_motor(steps_per_compartment);
        absolute_time_t time = make_timeout_time_ms(10);
        while(!sensor_triggered && !time_reached(time)){
            tight_loop_contents();
        }

        if(sensor_triggered){

            printf("Pill was released from the compartment:  #%d.\n", step_and_stop+1);
            sensor_triggered = false;
        }
        else{
            printf("Pill has not been dispensed from the compartment:  #%d.\n", step_and_stop+1);

            handle_error();
        }
        sleep_ms(2000);
        step_and_stop++;

    }
}


// Handle Error
void handle_error() {
    for (int blink = 0; blink < 5; blink++) {
        gpio_put(LED_1, true);
        sleep_ms(200);
        gpio_put(LED_1, false);
        sleep_ms(200);
    }
}