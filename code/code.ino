#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#define ANGLE_CORRECTOR 147
#define SERVO_PAUSE_TIME 20000

typedef struct {
    uint8_t ECHO_PIN;
    uint8_t TRIGGER_PIN;
    uint8_t ID;
} UltrasonicSensor;

typedef struct {
    uint8_t INPUT_PIN;
} ServoMotor;

UltrasonicSensor  US_1 = {PD2, PB3, 1};
UltrasonicSensor  US_2 = {PD3, PB5, 2};
ServoMotor        SERVO = {PB2};

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

void SERVO_init_timer1() {
    TCCR1A |= (1 << WGM11) | (1 << WGM10);
    TCCR1B |= (1 << WGM12) | (1 << CS11);
    TCCR1A |= (1 << COM1B1);
}

void SERVO_change_angle(int initial, int final) {
    int stepper = 1;
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
    } else {
      return;
    }
}

void SERVO_move_servo(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    int dutyCycle = (angle * 4) - ANGLE_CORRECTOR; //This value is a trial and error thing
    
    OCR1B = dutyCycle;
}

void GENERAL_init_ports() {
    // Outputs
    DDRB |= (1 << US_1.TRIGGER_PIN);
    DDRB |= (1 << US_2.TRIGGER_PIN);
    DDRB |= (1 << SERVO.INPUT_PIN);

    //Inputs
    DDRD &= ~(1 << US_1.ECHO_PIN);
    DDRD &= ~(1 << US_2.ECHO_PIN);
}

int main(void) {
    init(); //Need to remove this, Arduino library
    
    // Initialize ports, UART, and servo
    GENERAL_init_ports();
    UART_init();
    SERVO_init_timer1();
    
    while (1) {        
        UART_sendString("\n");
        SENSORS_measure_distance(US_1, true);
        SENSORS_measure_distance(US_2, true);

        SERVO_change_angle(0,180);
        SERVO_change_angle(180,0);
        
    }
    
    return 0;
}
