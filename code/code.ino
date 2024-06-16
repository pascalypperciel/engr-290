#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>

// Values to tweak
#define SENSOR_ITERATOR     10
#define DISTANCE_HORIZONTAL 40.00
#define DISTANCE_VERTICAL   100.00
#define SIDEWAYS_DELAY_MS   2000
#define DELAY_BEFORE_TURN   3000

// Constants
#define FAN_HORI_SPEED      255
#define FAN_UPHILL_SPEED    170
#define FAN_DOWNHILL_SPEED  170
#define FAN_MAX             255
#define FAN_OFF             0
#define SERVO_UPHILL        0
#define SERVO_SIDEWAYS      90
#define SERVO_DOWNHILL      180

// Debug
#define RUN_HOVERCRAFT      true
#define DEBUG_SENSORS       false
#define DEBUG_SERVO         false
#define DEBUG_FANS          false

// Structures
typedef struct {
    uint8_t ECHO_PIN;
    uint8_t TRIGGER_PIN;
    const char* ID;
} UltrasonicSensor;

typedef struct {
    uint8_t INPUT_PIN;
} ServoMotor;

typedef struct {
    uint8_t INPUT_PIN;
} FAN;

// Components
UltrasonicSensor  US_FRONT =  {PD2, PB3, "Front"};
UltrasonicSensor  US_SIDE =   {PD3, PB5, "Side"};
ServoMotor        SERVO =     {PB1};
FAN               FAN_LIFT =  {PD5};
FAN               FAN_STEER = {PD6};

// Functions
void UART_init() {
    unsigned int ubrr = F_CPU / 16 / 9600 - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void UART_send_char(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_send_string(const char *str) {
    while (*str) {
        UART_send_char(*str++);
    }
}

uint32_t SENSORS_measure_distance(UltrasonicSensor US, bool display_distance) {
    // Send a 10us pulse to trigger pin
    PORTB &= ~(1 << US.TRIGGER_PIN);
    _delay_us(2);
    PORTB |= (1 << US.TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << US.TRIGGER_PIN);

    // Measure the duration of the pulse from echo pin
    unsigned long duration = 0;
    unsigned long overflow_count = 0;
    unsigned long max_overflows = 0xFFFF / 256;

    while (!(PIND & (1 << US.ECHO_PIN))) {
        if (overflow_count >= max_overflows) return -1;
        _delay_us(1);
    }

    TCNT2 = 0;
    TCCR2B |= (1 << CS21);

    while (PIND & (1 << US.ECHO_PIN)) {
        if (TCNT2 == 0) {
            overflow_count++;
            if (overflow_count >= max_overflows) {
                TCCR2B &= ~(1 << CS21);
                return -1;
            }
        }
    }

    TCCR2B &= ~(1 << CS21);

    duration = overflow_count * 256 + TCNT2;
    float time_taken = duration * (8.0 / F_CPU);
    float distance = (time_taken * 34300.0);

    if (distance > 400.00) distance = 400.00;

    if (display_distance) {
        SENSORS_display_distances(distance, US.ID);
    }

    return distance;
}

void SENSORS_display_distances(float distance, int id) {
    char buffer[32];
    char distance_buffer[16];
    
    dtostrf(distance, 6, 2, distance_buffer);
    snprintf(buffer, sizeof(buffer), "%s = %s\r\t", id, distance_buffer);
    
    UART_send_string(buffer);
}

float SENSORS_distances_average(UltrasonicSensor US) {
    float sum = 0;
    for (int i = 0; i < SENSOR_ITERATOR; i++) {
        _delay_us(10000);
        float distance = SENSORS_measure_distance(US, false);
        sum += distance;
    }
    return sum / SENSOR_ITERATOR;
}

bool SENSORS_opening_detected() {
    return SENSORS_distances_average(US_SIDE) > DISTANCE_HORIZONTAL;
}

int SENSOR_decide_angle() {
    if (SENSORS_distances_average(US_FRONT) >= DISTANCE_VERTICAL) {
        return SERVO_UPHILL;
    } else {
        return SERVO_DOWNHILL;
    }
}

void SERVO_init_timer1() {
    TCCR1A = (1 << WGM11) | (1 << COM1A1);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    ICR1 = 39999;
}

void SERVO_change_angle(float angle) {
    if (RUN_HOVERCRAFT) {
        FAN_set_spin(FAN_STEER, FAN_OFF);
        _delay_ms(DELAY_BEFORE_TURN);      
    }
    
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    OCR1A = map(angle, 0, 180, 1000, 4500);
    
    if (RUN_HOVERCRAFT) {
        if (angle == SERVO_SIDEWAYS) {
            FAN_set_spin(FAN_STEER, FAN_HORI_SPEED);
        } else if (angle == SERVO_UPHILL) {
            FAN_set_spin(FAN_STEER, FAN_UPHILL_SPEED);
        } else {
            FAN_set_spin(FAN_STEER, FAN_DOWNHILL_SPEED);
        }
    }
}

void FAN_init() {
    TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
    TCCR0B |= (1 << CS01) | (1 << CS00);
}

void FAN_set_spin(FAN fan, int value) {
    if (fan.INPUT_PIN == PD5) {
        OCR0B = value;
    } else if (fan.INPUT_PIN == PD6) {
        OCR0A = value;
    }
}

void GENERAL_init_interrupts() {
    SREG |= (1 << 7);
}

void GENERAL_init_ports() {
    // Outputs
    DDRB |= (1 << US_FRONT.TRIGGER_PIN);
    DDRB |= (1 << US_SIDE.TRIGGER_PIN);
    DDRB |= (1 << SERVO.INPUT_PIN);
    DDRD |= (1 << FAN_LIFT.INPUT_PIN);
    DDRD |= (1 << FAN_STEER.INPUT_PIN);

    //Inputs
    DDRD &= ~(1 << US_FRONT.ECHO_PIN);
    DDRD &= ~(1 << US_SIDE.ECHO_PIN);
}

void GENERAL_setup() {
    FAN_set_spin(FAN_LIFT, FAN_OFF);
    FAN_set_spin(FAN_STEER, FAN_OFF);
    if (RUN_HOVERCRAFT) {
        SERVO_change_angle(SERVO_UPHILL);
        FAN_set_spin(FAN_STEER, FAN_HORI_SPEED);
    }
}

void GENERAL_move_sideways() {
    SERVO_change_angle(SERVO_SIDEWAYS);
    _delay_ms(SIDEWAYS_DELAY_MS);
    SERVO_change_angle(SENSOR_decide_angle());
}

int main(void) {
    // Initialize ports, UART, and servo
    GENERAL_init_interrupts();
    GENERAL_init_ports();
    UART_init();
    SERVO_init_timer1();
    FAN_init();
    GENERAL_setup();

    char buffer[16];
    while (1) {
        UART_send_string("\n");
        if (RUN_HOVERCRAFT && SENSORS_opening_detected()) {
            GENERAL_move_sideways();
        }

        if (DEBUG_SENSORS) {
            SENSORS_measure_distance(US_FRONT, true);
            SENSORS_measure_distance(US_SIDE, true);
        }

        if (DEBUG_SERVO) {
            snprintf(buffer, sizeof(buffer), "Angle = %d\r\t", OCR1A);
            UART_send_string(buffer);
        }

        if (DEBUG_FANS) {
            snprintf(buffer, sizeof(buffer), "Steer = %d\r\t", OCR0A);
            UART_send_string(buffer);
            snprintf(buffer, sizeof(buffer), "Lift = %d\r\t", OCR0B);
            UART_send_string(buffer);
        }
        
    }
    
    return 0;
}
