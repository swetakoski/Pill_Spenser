#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include <string.h>

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
#define ERROR_BLINKS 5       // Number of LED blinks for errors
#define LED_BLINK_DELAY 200  // LED blink duration
#define BUTTON_SLEEP 50      // button sleep length after press

// UART macros
#define UARTID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define STRLEN 200

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
    STATE_CHECK_LORA_CONNECTION,
    STATE_CONNECT_LORA_TO_NETWORK,
} dispenser_state;

dispenser_state current_state = STATE_CHECK_LORA_CONNECTION;

// Global Variables
bool sensor_triggered = false;
bool system_calibrated = false;

// Function Prototypes
void initialize_gpio(void);
void init_uart(void);
void switch_state(void);
void blink_led(void);
void led_controller(void);
void idle_mode(void);
void calibrate_motor(void);
void activate_motor_step(uint sleep);
void run_motor(int N);
void dispensing_pills(void);
void piezo_interrupt(uint gpio, uint32_t events);
void send_at(void);
void send_command(const char *command);
bool receive_answer(char *response);
void connect_lorawan(void);

int main() {
    stdio_init_all();
    initialize_gpio();
    init_uart();

    while (1) {
        switch_state();
    }
}

// GPIO Initialization
void initialize_gpio(void) {
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

// uart initialization
void init_uart(void) {
    uart_init(UARTID,BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void switch_state(void) {
    switch (current_state) {
        case STATE_CHECK_LORA_CONNECTION:
            send_at();
        	break;
        case STATE_CONNECT_LORA_TO_NETWORK:
            connect_lorawan();
        	break;
        case STATE_WAIT:
            idle_mode();
            break;
        case STATE_CALIBRATION:
            calibrate_motor();
            break;
        case STATE_DISPENSE_PILLS:
            dispensing_pills();
            break;
    }
}

// led blinks
void blink_led(void) {
    gpio_put(LED_1, true);
    sleep_ms(LED_BLINK_DELAY);
    gpio_put(LED_1, false);
    sleep_ms(LED_BLINK_DELAY);
}

// led controller
void led_controller(void) {
    while(gpio_get(SW_2) && gpio_get(SW_0)) {
        if(system_calibrated == false) {
            blink_led();
        } else {
            gpio_put(LED_1, true);
        }
    }
}

// Idle Mode
void idle_mode(void) {
    printf("Entering idle mode. ");
    if(system_calibrated == false) {
        printf("Press SW_2 to calibrate.\n");
    }else {
        printf("Press SW_0 to start pill dispenser or SW_2 to recalibrate.\n");
    }
    led_controller();
    if(!gpio_get(SW_2)) {
        sleep_ms(BUTTON_SLEEP);
        current_state = STATE_CALIBRATION;
    }
    if(!gpio_get(SW_0) && system_calibrated == true) {
        sleep_ms(BUTTON_SLEEP);
        gpio_put(LED_1, false);
        current_state = STATE_DISPENSE_PILLS;
    }else if(!gpio_get(SW_0) && system_calibrated == false) {
        sleep_ms(BUTTON_SLEEP);
        printf("The motor has not been calibrated. Calibrate first.\n");
        while(!gpio_get(SW_0)) {
            sleep_ms(BUTTON_SLEEP);
        }
    }
}

// Calibrate Motor
void calibrate_motor(void) {
    gpio_put(LED_1, true);
    printf("Calibrating motor...\n");
    int total_steps = 0;
    uint fork_hits = 0;
    bool previous_state = false;
    // find the first falling edge
    while (gpio_get(OPTO_FORK)) {
        activate_motor_step(5);
    }
    // run full 3 rounds and count steps
    while(fork_hits < 3) {
        activate_motor_step(4 - fork_hits);
        total_steps++;
        if(!gpio_get(OPTO_FORK) && previous_state) {
            fork_hits++;
        }
        previous_state = gpio_get(OPTO_FORK);
    }
    steps_per_revolution = total_steps / 3;
    // move motor a bit so dispenser wheel and dispenser base holes line up, value might depend on device
    for (int i = 0; i < 170; i++) {
        activate_motor_step(1);
    }
    printf("Motor has been calibrated.\n");
    system_calibrated = true;
    current_state = STATE_WAIT;
}

// Activate Motor Step
void activate_motor_step(uint sleep) {
    static int step_index = 0;
    step_index = (step_index + 1) % 8;
    gpio_put(IN1, half_step_sequence[step_index][0]);
    gpio_put(IN2, half_step_sequence[step_index][1]);
    gpio_put(IN3, half_step_sequence[step_index][2]);
    gpio_put(IN4, half_step_sequence[step_index][3]);
    sleep_ms(sleep);
}

// Run Motor
void run_motor(int N) {
    for (int i = 0; i < N; i++) {
        activate_motor_step(2);
    }
}

// Dispensing Pills
void dispensing_pills(void) {
    printf("Dispense button pressed. Dispensing pills...\n");
    int pill_drops = 0;
    int step_and_stop = 0;
    int steps_per_compartment = steps_per_revolution / 8; // Calculate steps per compartment
    while(step_and_stop < 7){
        sensor_triggered = false;
        // Rotate motor to the next compartment
        run_motor(steps_per_compartment);
        absolute_time_t time = make_timeout_time_ms(100);
        while(!sensor_triggered && !time_reached(time)){
            tight_loop_contents();
        }
        if(sensor_triggered){
            printf("Pill was released from the compartment:  #%d.\n", step_and_stop+1);
            sensor_triggered = false;
            pill_drops++;
        }
        else{
            printf("Pill wasn't dispensed from the compartment:  #%d.\n", step_and_stop+1);
            for (int blink = 0; blink < 5; blink++) {
                blink_led();
            }
        }
        sleep_ms(30000);
        step_and_stop++;
    }
    if(pill_drops == 7) {
        printf("All pills have been dispensed, going back to idle state.\n");
    } else if (pill_drops > 0 && pill_drops < 7) {
        printf("Pills have been dispensed from %d compartments, going back to idle state.\n", pill_drops);
    } else {
        printf("No pills have been dispensed, going back to idle state.\n");
    }
    current_state = STATE_WAIT;
}

// Interrupt Callback for Piezo Sensor
void piezo_interrupt(uint gpio, uint32_t events) {
    if (gpio == PIEZO_SENSOR && events & GPIO_IRQ_EDGE_FALL) {
        sensor_triggered = true;
    }
}

// function to send command to lora
void send_command(const char *command) {
    uart_write_blocking(UARTID, (uint8_t *)command, strlen(command));
}

// function to get and handle answer from lora
bool receive_answer(char *response) {
    bool answer_received = false;
    int i = 0;

    // attempt reading uart within 500 ms
    while(uart_is_readable_within_us(UARTID, 500000) && !answer_received) {
        char c = uart_getc(UARTID);
        if (c == '\r' || c == '\n') {
            response[i] = '\0';
            answer_received = true;
            printf("%s\n", response);
            if (c == '\r') {
                uart_getc(UARTID);
            }
        } else {
            if(i < STRLEN - 1) {
                response[i++] = c;
            }
        }
    }
    return answer_received;
}

// function to send AT
void send_at(void) {
    printf("Checking connection to LoRa module...\n");
    int attempts = 0;

    // try to send AT max 5 times
    while(attempts < 5) {
        char response[STRLEN];
        send_command("AT\r\n");
        receive_answer(response);

        // if there's the right answer stop the loop and move to state 3
        if(strcmp(response, "+AT: OK") == 0) {
            printf("Connected to LoRa module\n");
            current_state = STATE_CONNECT_LORA_TO_NETWORK;
            return;
        }
        attempts++;
    }
    printf("Module not responding\n");
    current_state = STATE_WAIT;
}

void connect_lorawan() {
    char response[STRLEN];
    send_command("AT+MODE=LWOTAA\r\n");
    receive_answer(response);
    send_command("AT+KEY=APPKEY,\"b903ad771f90c92de88a8a1eecbb2404\"\r\n");
    receive_answer(response);
    send_command("AT+CLASS=A\r\n");
    receive_answer(response);
    send_command("AT+PORT=8\r\n");
    receive_answer(response);
    send_command("AT+JOIN\r\n");
    receive_answer(response);
    receive_answer(response);
    sleep_ms(20000);
    receive_answer(response);
    receive_answer(response);
    receive_answer(response);
    send_command("AT+MSG=\"MESSAGE SENT\"\r\n");
    receive_answer(response);
    current_state = STATE_WAIT;
}
