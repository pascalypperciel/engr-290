#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include "mpu6050.h"

// Values to tweak
#define RANGE_WALL          50
#define ERROR_TURN_YAW      30
#define WAIT_AFTER_TURN     50
#define VALUES_IN_A_ROW     2

// Constants
#define FAN_SPEED_MAX       255
#define FAN_SPEED_LESS      185
#define FAN_SPEED_LESS_LESS 155
#define FAN_SPEED_OFF       0

#define ANGLE_SERVO_RIGHT   170
#define ANGLE_SERVO_CENTER  90
#define ANGLE_SERVO_LEFT    15

#define ANGLE_YAW_AWAY      0
#define ANGLE_YAW_TOWARDS   180

#define MILLIS_INC 1
#define FRACT_INC 3
#define FRACT_MAX 1000

// Structures
typedef struct {
    uint8_t ECHO_PIN;
    uint8_t TRIGGER_PIN;
} UltrasonicSensor;

typedef struct {
    uint8_t INPUT_PIN;
} ServoMotor;

typedef struct {
    uint8_t INPUT_PIN;
} Fan;

typedef struct {
    uint8_t SDA_PIN;
    uint8_t SCL_PIN;
} Gyroscope;

// Components
UltrasonicSensor  US_RIGHT =  {PD2, PB3};
UltrasonicSensor  US_LEFT =   {PD3, PB5};
ServoMotor        SERVO =     {PB1};
Fan               FAN_LIFT =  {PD5};
Fan               FAN_STEER = {PD6};
Gyroscope         IMU =       {PC4, PC5};  
MPU6050           ICU;

// Global variable
int _target_yaw = ANGLE_YAW_AWAY;
int _counter = WAIT_AFTER_TURN;
int _opening_in_a_row_right = 0;
int _opening_in_a_row_left = 0;
bool _last_was_big_right = false;
bool _last_was_big_left = false;
float _d_last = 0;
volatile unsigned long _timer0_millis = 0;
volatile unsigned char _timer0_fract = 0;
static unsigned long _timer0_overflow_count = 0;

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

float SENSORS_measure_distance(UltrasonicSensor US) {
    // Send a 10us pulse to trigger pin
    PORTB &= ~(1 << US.TRIGGER_PIN);
    _delay_us(2);
    PORTB |= (1 << US.TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << US.TRIGGER_PIN);

    long distance = 0;
    long loop__counter = 0;
    long loop_max = (23529 * (F_CPU / 1000000)) / 16;
    bool break_all = false;

    while ((PIND & (1 << US.ECHO_PIN)) == (1 << US.ECHO_PIN)) {
        if (loop__counter++ == loop_max) {
            break_all = true;
            break;
        }
    }

    while ((PIND & (1 << US.ECHO_PIN)) != (1 << US.ECHO_PIN) && !break_all) {
        if (loop__counter++ == loop_max) {
            break_all = true;
            break;
        }
    }

    while ((PIND & (1 <<US.ECHO_PIN)) == (1 << US.ECHO_PIN) && !break_all) {
        if (loop__counter++ == loop_max) {
            break;
        }
        distance++;
    }

    return (float)(distance * (16 / (F_CPU / 1000000))) / 58.8235;
}

int SENSORS_opening_detected() {
    char buffer[32];
    
    int right = (int) SENSORS_measure_distance(US_RIGHT);
    snprintf(buffer, sizeof(buffer), "Right = %d\r\t", right);
    UART_send_string(buffer);
    
    int left = (int) SENSORS_measure_distance(US_LEFT);
    snprintf(buffer, sizeof(buffer), "Left = %d\r\t", left);
    UART_send_string(buffer);
    

    if (left >= RANGE_WALL) {
        _opening_in_a_row_left++;
        if (_opening_in_a_row_left == VALUES_IN_A_ROW && _last_was_big_left) {
            _opening_in_a_row_left = 0;
            _last_was_big_right = false;
            return ANGLE_SERVO_LEFT;
        } else {
            _last_was_big_left = true;
            return -1;
        }
    }
    if (right >= RANGE_WALL) {
        _opening_in_a_row_right++;
        if (_opening_in_a_row_right == VALUES_IN_A_ROW && _last_was_big_right) {
            _opening_in_a_row_right = 0;
            _last_was_big_right = false;
            return ANGLE_SERVO_RIGHT;
        } else {
            _last_was_big_right = true;
            return -1;
        }
    }
    _last_was_big_right = false;
    _last_was_big_left = false;
    _opening_in_a_row_right = 0;
    _opening_in_a_row_left = 0;
    return -1;
}

void SERVO_init_timer1() {
    TCCR1A = (1 << WGM11) | (1 << COM1A1);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    ICR1 = 39999;
}

void SERVO_change_angle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    OCR1A = map(angle, 0, 180, 800, 4300);
}
//
//void FAN_init() {
//    TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
//    TCCR0B |= (1 << CS01) | (1 << CS00);
//}
//
//void FAN_set_spin(FAN fan, int value) {
//    if (fan.INPUT_PIN == PD5) {
//        OCR0B = value;
//    } else if (fan.INPUT_PIN == PD6) {
//        OCR0A = value;
//    }
//}

void MPU_change_target_yaw() {
    if (_target_yaw == ANGLE_YAW_AWAY) {
        _target_yaw = ANGLE_YAW_TOWARDS;
    } else {
        _target_yaw = ANGLE_YAW_AWAY;
    }
}

bool MPU_is_turn_over() {
    float current_yaw = MPU_get_yaw_turning();
    return current_yaw >= _target_yaw - ERROR_TURN_YAW && current_yaw <= _target_yaw + ERROR_TURN_YAW;
}

float MPU_get_yaw() {
    MPU6050_t data = ICU.get();
    char buffer[16];
    float yaw = (int) data.dir.yaw;
    dtostrf(yaw, 6, 2, buffer);
    UART_send_string("\nYaw : \t");
    UART_send_string(buffer);
    return yaw;
}

float MPU_get_yaw_turning() {
    MPU6050_t data = ICU.get();
    char buffer[32];
    char _target_yaw_buffer[16];
    
    int yaw = (int) abs(data.dir.yaw);
    if (yaw >= 360 || yaw <= -360) {
        ICU.begin();
        ICU.get();
        yaw = (int) abs(data.dir.yaw);
    }
    
    dtostrf(yaw, 6, 2, buffer);
    dtostrf(_target_yaw, 6, 2, _target_yaw_buffer);
    
    strcat(buffer, " _target_yaw: ");
    strcat(buffer, _target_yaw_buffer);
    
    UART_send_string("\nYee: \t");
    UART_send_string(buffer);
    
    return yaw;
}

void I2C_init() {
    TWSR = 0x00;
    TWBR = ((F_CPU / 100000L) - 16) / 2;
    TWCR = (1 << TWEN);
}

void I2C_start() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_stop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}

bool I2C_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return (TWSR & 0xF8) == TW_MT_DATA_ACK;
}

uint8_t I2C_read(bool ack) {
    if (ack) {
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
    } else {
        TWCR = (1 << TWINT) | (1 << TWEN);
    }
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

bool IMU_init() {
    if (IMU_absent() || IMU_wake() || IMU_set_rate_divider() || IMU_set_acceleration_range() ||
        IMU_set_gyroscope_range() || IMU_set_DLPF_bandwidth() || IMU_calibrate()) {
            return false;
        }

    int error = IMU_calibrate()
}

unsigned long GENERAL_millis()
{
    unsigned long m;
    uint8_t oldSREG = SREG;
    
    cli();
    m = _timer0_millis;
    SREG = oldSREG;

    return m;
}

void GENERAL_init_timer0() {
    TCCR0A = 0;
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 = (1 << TOIE0);
    sei();
}

void GENERAL_init_interrupts() {
    SREG |= (1 << 7);
}

void GENERAL_init_ports() {
    // Outputs
    DDRB |= (1 << US_RIGHT.TRIGGER_PIN);
    DDRB |= (1 << US_LEFT.TRIGGER_PIN);
    DDRB |= (1 << SERVO.INPUT_PIN);
    DDRC |= (1 << IMU.SDA_PIN);
    DDRC |= (1 << IMU.SCL_PIN);
    DDRD |= (1 << FAN_LIFT.INPUT_PIN);
    DDRD |= (1 << FAN_STEER.INPUT_PIN);

    // Inputs
    DDRD &= ~(1 << US_RIGHT.ECHO_PIN);
    DDRD &= ~(1 << US_LEFT.ECHO_PIN);

    // Set high
    PORTC |= (1 << IMU.SDA_PIN);
    PORTC |= (1 << IMU.SCL_PIN);
}

void GENERAL_components_setup() {
    SERVO_change_angle(ANGLE_SERVO_CENTER);
//    FAN_set_spin(FAN_LIFT, FAN_SPEED_MAX);
//    FAN_set_spin(FAN_STEER, FAN_SPEED_LESS);
}

void GENERAL_go_forward() {
    float imu_error;
    float yaw = MPU_get_yaw();
    float PID_constant = 0;
    if (_target_yaw == ANGLE_YAW_AWAY) {
      imu_error = 90 - (_target_yaw - yaw);
      PID_constant = _target_yaw - yaw;
      float d_input = PID_constant - _d_last;
      _d_last = PID_constant;             
      imu_error += (-(PID_constant * 0.5 + d_input * 2));
    }
    else if (_target_yaw == ANGLE_YAW_TOWARDS) {
      imu_error = 90 - (_target_yaw + yaw);      //(180 + 171) - 270 = 81
      PID_constant = _target_yaw + yaw;          //180 -171 = 9 
          if(abs(PID_constant) > 5) {
            float d_input = PID_constant - _d_last;
            _d_last = PID_constant;                  
            imu_error += (PID_constant * 0.5 + d_input * 2);
          }
    }
    
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "\tservo = %d\r\t", (int)imu_error);
    UART_send_string(buffer);
    
    SERVO_change_angle(imu_error);
}

void GENERAL_make_turn(float opening_angle) {
//    FAN_set_spin(FAN_STEER, FAN_SPEED_LESS_LESS);
    SERVO_change_angle(opening_angle);
    MPU_change_target_yaw();
    while (!MPU_is_turn_over());
//    FAN_set_spin(FAN_STEER, FAN_SPEED_LESS);
    SERVO_change_angle(90);
    _delay_ms(100);
}

int main(void) {
    GENERAL_init_timer0();
    GENERAL_init_interrupts();
    GENERAL_init_ports();
    UART_init();
    SERVO_init_timer1();
//    FAN_init();
    GENERAL_TWI_init();
    int error = ICU.begin();
    if (error) { UART_send_string("MPU initialization failed :("); }
    GENERAL_components_setup();
    
    _counter = WAIT_AFTER_TURN;
    _opening_in_a_row_right = 0;
    _opening_in_a_row_left = 0;
    _last_was_big_right = false;
    _last_was_big_left = false;

    while (1) {
        if (_counter > 0) {
            _counter--;   
        }
        GENERAL_forward_logic();
    
        int opening_angle = SENSORS_opening_detected();
        if (opening_angle != -1 && _counter == 0) {
            GENERAL_make_turn(opening_angle);
            _counter = WAIT_AFTER_TURN;
            _can_i_turn_yet = 0;
        }
    }
}

SIGNAL(TIMER0_OVF_vect)
{
    unsigned long m = _timer0_millis;
    unsigned char f = _timer0_fract;

    m += MILLIS_INC;
    f += FRACT_INC;
    if (f >= FRACT_MAX) {
        f -= FRACT_MAX;
        m += 1;
    }

    _timer0_fract = f;
    _timer0_millis = m;
    _timer0_overflow_count++;
}
