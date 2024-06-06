#ifndef F_CPU
#define F_CPU 16000000
#endif // F_CPU

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>

#define US_1_TRIGGER_PIN  PB3
#define US_1_ECHO_PIN     PD2
#define US_2_TRIGGER_PIN  PB5
#define US_2_ECHO_PIN     PD3

void UART_init(unsigned int baud) {
    unsigned int ubrr = F_CPU/16/baud-1;
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void UART_sendChar(unsigned char data) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void UART_sendString(const char *str) {
    while (*str) {
        UART_sendChar(*str++);
    }
}

void init_ports() {
    DDRB |= (1 << US_1_TRIGGER_PIN);
    DDRB |= (1 << US_2_TRIGGER_PIN);
    DDRD &= ~(1 << US_1_ECHO_PIN);
    DDRD &= ~(1 << US_2_ECHO_PIN);
}

uint32_t measure_distance(int TRIG_PIN, int ECHO_PIN) {
    // Send a 10us pulse to trigger pin
    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    // Measure the duration of the pulse from echo pin
    unsigned long duration = 0;
    unsigned long max_duration = F_CPU / 10;
    while (!(PIND & (1 << ECHO_PIN))) {
        if (duration++ >= max_duration) return -1;
        _delay_us(1);
    }
    TCNT1 = 0;
    TCCR1B |= (1 << CS11);
    while (PIND & (1 << ECHO_PIN)) {
        if (TCNT1 > max_duration) return -1;
    }
    TCCR1B &= ~(1 << CS11);

    // Calculate distance in centimeters
    float time_taken = TCNT1 * (8.0 / F_CPU);
    float distance = (time_taken * 34300.0) / 2.0;

    return distance;
}

void display_distances() {
    float distance1, distance2;
    char buffer[32];
    
    distance1 = measure_distance(US_1_TRIGGER_PIN, US_1_ECHO_PIN);
    if (distance1 == -1) {
        _delay_ms(500);
        return;
    }
    char distance_buffer[16];
    dtostrf(distance1, 6, 2, distance_buffer);
    snprintf(buffer, sizeof(buffer), "\nDistance 1 = %s cm\r\t", distance_buffer);
    UART_sendString(buffer);
    
    distance2 = measure_distance(US_2_TRIGGER_PIN, US_2_ECHO_PIN);
    if (distance2 == -1) {
        _delay_ms(500);
        return;
    }
    distance_buffer[16];
    dtostrf(distance2, 6, 2, distance_buffer);
    snprintf(buffer, sizeof(buffer), "Distance 2 = %s cm\r", distance_buffer);
    UART_sendString(buffer);
}

int main(void) {
    // Initialize ports and UART
    init_ports();
    UART_init(9600);
    
    while (1) {
      display_distances();
    }
    
    return 0;
}
