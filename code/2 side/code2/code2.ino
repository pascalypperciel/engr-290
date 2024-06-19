#include "mpu6050.h"
#include "HCSR04.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <Wire.h>

// Values to tweak
#define RANGE_WALL          40
#define ERROR_TURN_YAW      0
#define WAIT_AFTER_TURN     40
#define ITERATOR_AVERAGE    1
#define VALUES_IN_A_ROW     5


// Constants
#define FAN_SPEED_MAX       255
#define FAN_SPEED_LESS      230
#define FAN_SPEED_OFF       0

#define ANGLE_SERVO_RIGHT   180
#define ANGLE_SERVO_CENTER  90
#define ANGLE_SERVO_LEFT    0

#define ANGLE_YAW_AWAY      0
#define ANGLE_YAW_TOWARDS   180

#define PID_PROP            0.5

// Structures
typedef struct {
    uint8_t INPUT_PIN;
} ServoMotor;

typedef struct {
    uint8_t INPUT_PIN;
} FAN;

// Components
ServoMotor        SERVO =     {PB1};
FAN               FAN_LIFT =  {PD5};
FAN               FAN_STEER = {PD6};
MPU6050           ICU;
HCSR04            US_LEFT(11, 2);
HCSR04            US_RIGHT(13, 3);

// Global variable
int target_yaw = ANGLE_YAW_AWAY;
int counter = WAIT_AFTER_TURN;
int opening_in_a_row_right = 0;
int opening_in_a_row_left = 0;
bool last_was_big_right = false;
bool last_was_big_left = false;

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

int SENSORS_opening_detected() {
    char buffer[32];
    
    int right = (int) US_RIGHT.dist();
    snprintf(buffer, sizeof(buffer), "Right = %d\r\t", right);
    UART_send_string(buffer);
    
    int left = (int) US_LEFT.dist();
    snprintf(buffer, sizeof(buffer), "Left = %d\r\t", left);
    UART_send_string(buffer);
    
    if (right >= RANGE_WALL) {
        opening_in_a_row_right++;
        if (opening_in_a_row_right == VALUES_IN_A_ROW && last_was_big_right) {
            opening_in_a_row_right = 0;
            last_was_big_right = false;
            return ANGLE_SERVO_RIGHT;
        } else {
            last_was_big_right = true;
            return -1;
        }
    } else if (left >= RANGE_WALL) {
        opening_in_a_row_left++;
        if (opening_in_a_row_left == VALUES_IN_A_ROW && last_was_big_left) {
            opening_in_a_row_left = 0;
            return ANGLE_SERVO_LEFT;
        } else {
            last_was_big_left = true;
            last_was_big_left = false;
            return -1;
        }
    }
    last_was_big_right = false;
    last_was_big_left = false;
    opening_in_a_row_right = 0;
    opening_in_a_row_left = 0;
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

void MPU_change_target_yaw() {
    if (target_yaw == ANGLE_YAW_AWAY) {
        target_yaw = ANGLE_YAW_TOWARDS;
    } else {
        target_yaw = ANGLE_YAW_AWAY;
    }
}

bool MPU_is_turn_over() {
      float current_yaw = MPU_get_yaw_turning();
      return current_yaw >= target_yaw - ERROR_TURN_YAW && current_yaw <= target_yaw + ERROR_TURN_YAW;
}

float MPU_get_yaw() {
      MPU6050_t data = ICU.get();
      char buffer[16];
      float yaw = (int) data.dir.yaw;
//      if (yaw >= 360 || yaw <= -360) {
//          ICU.begin();
//          ICU.get();
//          yaw = (int) abs(data.dir.yaw);
//      }
      dtostrf(yaw, 6, 2, buffer);
      UART_send_string("\nYaw : \t");
      UART_send_string(buffer);
      return yaw;
}

float MPU_get_yaw_turning() {
      MPU6050_t data = ICU.get();
      char buffer[32];
      char target_yaw_buffer[16];
      
      int yaw = (int) abs(data.dir.yaw);
      if (yaw >= 360 || yaw <= -360) {
          ICU.begin();
          ICU.get();
          yaw = (int) abs(data.dir.yaw);
      }
  
      dtostrf(yaw, 6, 2, buffer);
      dtostrf(target_yaw, 6, 2, target_yaw_buffer);
  
      strcat(buffer, " target_yaw: ");
      strcat(buffer, target_yaw_buffer);
  
      UART_send_string("\nYee: \t");
      UART_send_string(buffer);
      
      return yaw;
}

void GENERAL_init_interrupts() {
    SREG |= (1 << 7);
}

void GENERAL_init_ports() {
    DDRB |= (1 << SERVO.INPUT_PIN);
    DDRD |= (1 << FAN_LIFT.INPUT_PIN);
    DDRD |= (1 << FAN_STEER.INPUT_PIN);
}

void GENERAL_components_setup() {
    SERVO_change_angle(ANGLE_SERVO_CENTER);
    FAN_set_spin(FAN_LIFT, FAN_SPEED_MAX);
    FAN_set_spin(FAN_STEER, FAN_SPEED_LESS);
}

void GENERAL_forward_logic() {
    //this code needs to take 90 (away) or 270 (towards) yaw and orient towards that, ideally the servo stays near 90. This will run until opening is detected.
    float imu_error;
    float yaw = MPU_get_yaw();
    if (target_yaw == ANGLE_YAW_AWAY) {
      imu_error = 90 - (target_yaw - yaw);
    }
    else if (target_yaw == ANGLE_YAW_TOWARDS) {
      imu_error = 90 - (target_yaw - yaw);
    }
    
    float PID_constant = target_yaw - yaw;
    if(abs(PID_constant) > 5) {
      imu_error += (-PID_constant);
    }
    
    SERVO_change_angle(imu_error);
}

void GENERAL_turn(float opening_angle) {
    MPU_change_target_yaw();
    SERVO_change_angle(opening_angle);
    while (!MPU_is_turn_over()) {
        delay(5);
    }
}

void setup() {
    GENERAL_init_interrupts();
    GENERAL_init_ports();
    UART_init();
    SERVO_init_timer1();
    FAN_init();
    Wire.begin();
    int error= ICU.begin();
    if (error) { UART_send_string("MPU initialization failed :("); }
    GENERAL_components_setup();
    counter = WAIT_AFTER_TURN;
    opening_in_a_row_right = 0;
    opening_in_a_row_left = 0;
    last_was_big_right = false;
    last_was_big_left = false;
}

void loop() {  
    if (counter > 0) {
        counter--;   
    }
    GENERAL_forward_logic();
    int opening_angle = SENSORS_opening_detected();
    if (opening_angle != -1 && counter == 0) {
        UART_send_string("TURNIIINNGGG===============");
        GENERAL_turn(opening_angle);
        counter = WAIT_AFTER_TURN;
    }
}
