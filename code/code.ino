#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>

// Values to tweak
#define ANGLE_CORRECTOR     147
#define SENSOR_ITERATOR     10
#define SERVO_PAUSE_TIME    20000
#define PRINT_OUTPUT        true
#define DISTANCE_HORIZONTAL 20.00
#define DISTANCE_VERTICAL   40.00
#define INITIAL_PAUSE_MS    4000
#define SIDEWAYS_DELAY_MS   3000

// Constants
#define FAN_ON            255
#define FAN_OFF           0
#define SENSOR_HORIZONTAL 0
#define SENSOR_VERTICAL   1
#define SERVO_FORWARD     0
#define SERVO_SIDEWAYS    90
#define SERVO_BACKWARD    180

// Structures
typedef struct {
    uint8_t ECHO_PIN;
    uint8_t TRIGGER_PIN;
    uint8_t ID;
} UltrasonicSensor;

typedef struct {
    uint8_t INPUT_PIN;
} ServoMotor;

typedef struct {
    uint8_t INPUT_PIN;
} FAN;

// Components
UltrasonicSensor  US_FRONT =  {PD2, PB3, 1};
UltrasonicSensor  US_SIDE =   {PD3, PB5, 2};
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

void UART_sendChar(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_sendString(const char *str) {
    while (*str) {
        UART_sendChar(*str++);
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
    snprintf(buffer, sizeof(buffer), "Distance %d = %s cm\r\t", id, distance_buffer);
    
    UART_sendString(buffer);
}

float SENSORS_distances_average(UltrasonicSensor US) {
    float sum = 0;
    for (int i = 0; i < SENSOR_ITERATOR; i++) {
        float distance = SENSORS_measure_distance(US, PRINT_OUTPUT);
        sum += distance;
    }
    return sum / SENSOR_ITERATOR;
}

bool SENSORS_opening_detected() {
    return SENSORS_distances_average(US_SIDE) > DISTANCE_HORIZONTAL;
}

int SENSOR_decide_angle() {
    if (SENSORS_distances_average(US_FRONT) >= DISTANCE_VERTICAL) {
        return SERVO_FORWARD;
    } else {
        return SERVO_BACKWARD;
    }
}

void SERVO_init_timer1() {
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);
    TCCR1A |= (1 << WGM11) | (1 << WGM10);
    TCCR1B |= (1 << WGM12) | (1 << CS11);
    TCCR1A |= (1 << COM1A1);
    
    #if F_CPU >= 8000000L
    TCCR1B |= (1 << CS10);
    #endif
}

void SERVO_change_angle(int initial, int final) {
    if (initial == final) return;
    FAN_set_spin(FAN_STEER, FAN_OFF);
    _delay_ms(1000);
    
    if (initial > final) {
        for (int pos = initial; pos >= final; pos--) {
            SERVO_move_servo(pos);
            _delay_us(SERVO_PAUSE_TIME);
        }
    } else if (initial < final) {
        for (int pos = initial; pos <= final; pos++) {
            SERVO_move_servo(pos);
            _delay_us(SERVO_PAUSE_TIME);
        }
    }
    
    FAN_set_spin(FAN_STEER, FAN_ON);
}

void SERVO_move_servo(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    int dutyCycle = (angle * 4) - ANGLE_CORRECTOR;
    
    OCR1A = dutyCycle;
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
    SERVO_change_angle(1, 0);
    FAN_set_spin(FAN_LIFT, FAN_ON);
    _delay_ms(INITIAL_PAUSE_MS);
}

int GENERAL_move_sideways(int current_servo_angle) {
    SERVO_change_angle(current_servo_angle, SERVO_SIDEWAYS);
    _delay_ms(SIDEWAYS_DELAY_MS);
    int direction = SENSOR_decide_angle();
    SERVO_change_angle(SERVO_SIDEWAYS, direction);
    return direction;
}

int main(void) {
    // Initialize ports, UART, and servo
    GENERAL_init_interrupts();
    GENERAL_init_ports();
    UART_init();
    SERVO_init_timer1();
    FAN_init();
    GENERAL_setup();

    int current_servo_angle = 0;
    while (1) {
        if (SENSORS_opening_detected) {
            current_servo_angle = GENERAL_move_sideways(current_servo_angle);
        }
    }
    
    return 0;
}
