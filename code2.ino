#include <mpu6050.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <Wire.h>

// Values to tweak
#define SENSOR_ITERATOR     10
#define DISTANCE_HORIZONTAL 70.00
#define DISTANCE_VERTICAL   80.00
#define SIDEWAYS_DELAY_MS   2000
#define DELAY_BEFORE_TURN   2000
#define ERROR_TURN_YAW      15

// Constants
#define FAN_HORI_SPEED      255
#define FAN_UPHILL_SPEED    170
#define FAN_DOWNHILL_SPEED  200
#define FAN_MAX             255
#define FAN_OFF             0

#define ANGLE_SERVO_RIGHT   180
#define ANGLE_SERVO_CENTER  90
#define ANGLE_SERVO_LEFT    0

#define ANGLE_YAW_AWAY      90
#define ANGLE_YAW_TOWARDS   270

#define PID_PROP 0.5
#define PID_INTE 0.1
#define PID_DIFF 2.0

// Debug
#define RUN_HOVERCRAFT      true
#define DEBUG_SENSORS       true
#define DEBUG_SERVO         false
#define DEBUG_FANS          false
#define DEBUG_LIFT          false
#define DEBUG_SERVO_SPIN    false
#define DEBUG_SKIRT         false

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
UltrasonicSensor  US_LEFT =  {PD2, PB3, "Left"};
UltrasonicSensor  US_RIGHT =   {PD3, PB5, "Right"};
ServoMotor        SERVO =     {PB1};
FAN               FAN_LIFT =  {PD5};
FAN               FAN_STEER = {PD6};
MPU6050           mpu;
unsigned long timer = 0; // variable to clock serial monitor

// Global variables
float i_input = 0;
float d_last = 0;
int target_yaw = ANGLE_YAW_AWAY;

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
    if (distance < 2.00) distance = 2.00;

    if (display_distance) {
        //SENSORS_display_distances(distance, US.ID);
    }

    return distance;
}

float SENSORS_distances_average(UltrasonicSensor US) {
    float sum = 0;
    for (int i = 0; i < SENSOR_ITERATOR; i++) {
        float distance = SENSORS_measure_distance(US, false);
        sum += distance;
    }
    return sum / SENSOR_ITERATOR;
}

void SENSORS_display_distances(float distance, int id) {
    char buffer[32];
    char distance_buffer[16];
    
    dtostrf(distance, 6, 2, distance_buffer);
    snprintf(buffer, sizeof(buffer), "%s = %s\r\t", id, distance_buffer);
    
    UART_send_string(buffer);
}

int SENSORS_opening_detected() {
    if (SENSORS_distances_average(US_RIGHT) > DISTANCE_HORIZONTAL) {
        return ANGLE_SERVO_RIGHT;
    } else if (SENSORS_distances_average(US_LEFT) > DISTANCE_HORIZONTAL) {
        return ANGLE_SERVO_LEFT;
    }
    return 0;
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

float PID_adjust(float error) {
    float pid_pset = error;
    float pid_iset = constrain(i_input + error, -50, +50);
    float pid_dset = error - d_last;
    d_last = error;
  
    return pid_pset * PID_PROP + pid_iset * PID_INTE + pid_dset * PID_DIFF;
}

void MPU_change_target_yaw() {
    if (target_yaw == ANGLE_YAW_AWAY) {
        target_yaw = ANGLE_YAW_TOWARDS;
    } else {
        target_yaw = ANGLE_YAW_AWAY;
    }
}

bool MPU_is_turn_over() {
      float current_yaw = MPU_get_yawss();
      return current_yaw >= target_yaw - ERROR_TURN_YAW && current_yaw <= target_yaw + ERROR_TURN_YAW;
}

float MPU_get_yaw() {
  MPU6050_t data = mpu.get();
  return data.dir.yaw;
}

float MPU_get_yaws() {
      float yaw = MPU_get_yaw();
      char buffer[16];
      dtostrf(yaw, 6, 2, buffer);
      UART_send_string("\nYaw : ");
      UART_send_string(buffer);
      return yaw;
}

float MPU_get_yawss() {
    float yaw = MPU_get_yaw();
    char buffer[32]; // Increase buffer size to accommodate both values
    char target_yaw_buffer[16]; // Buffer for target_yaw

    dtostrf(yaw, 6, 2, buffer); // Convert yaw to string
    dtostrf(target_yaw, 6, 2, target_yaw_buffer); // Convert target_yaw to string

    strcat(buffer, " target_yaw: "); // Append the label for target_yaw
    strcat(buffer, target_yaw_buffer); // Append the target_yaw value

    UART_send_string("\nYee: ");
    UART_send_string(buffer);

    return yaw;
}

void GENERAL_init_interrupts() {
    SREG |= (1 << 7);
}

void GENERAL_init_ports() {
    // Outputs
    DDRB |= (1 << US_RIGHT.TRIGGER_PIN);
    DDRB |= (1 << US_LEFT.TRIGGER_PIN);
    DDRB |= (1 << SERVO.INPUT_PIN);
    DDRD |= (1 << FAN_LIFT.INPUT_PIN);
    DDRD |= (1 << FAN_STEER.INPUT_PIN);

    // Inputs
    DDRD &= ~(1 << US_RIGHT.ECHO_PIN);
    DDRD &= ~(1 << US_LEFT.ECHO_PIN);
}

void GENERAL_setup() {
    FAN_set_spin(FAN_LIFT, FAN_OFF);
    FAN_set_spin(FAN_STEER, FAN_OFF);
    if (RUN_HOVERCRAFT) {
        SERVO_change_angle(ANGLE_SERVO_CENTER);
        FAN_set_spin(FAN_LIFT, FAN_MAX);
        FAN_set_spin(FAN_STEER, FAN_MAX);
    }
  int error= mpu.begin(); 
}

void GENERAL_turn(float opening_angle) {
    MPU_change_target_yaw();
    SERVO_change_angle(opening_angle);
    while (!MPU_is_turn_over()) {
        delay(5);
    }
}

void forward_logic() {
  //this code needs to take 90 (away) or 270 (towards) yaw and orient towards that, ideally the servo stays near 90. This will run until opening is detected.
  float imu_error;
  if (target_yaw == ANGLE_YAW_AWAY) {
    imu_error = 180 - (target_yaw - MPU_get_yaws());
  }
  else if (target_yaw == ANGLE_YAW_TOWARDS) {
    imu_error = 180 - (target_yaw - MPU_get_yaws() - 180);
  }
  if(imu_error > 180) { imu_error = 180; }
  if(imu_error < 0) { imu_error = 0; }
  SERVO_change_angle(imu_error);
}

void setup() {
    GENERAL_init_interrupts();
    GENERAL_init_ports();
    UART_init();
    SERVO_init_timer1();
    FAN_init();
    GENERAL_setup();
    Wire.begin();
    byte status = mpu.begin();
    if (status != 0) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "MPU initialization failed with code %d\r\n", status);
        UART_send_string(buffer);
        while (1); // halt the program
    }
    UART_send_string("TESTING\n");
}

void loop() {     
    char buffer[50];
    if (RUN_HOVERCRAFT) {
        forward_logic();
        int opening_angle = SENSORS_opening_detected();
        if(opening_angle != 0) {
        UART_send_string("TURNINGGGGG");
        GENERAL_turn(opening_angle);
        }
    }
    
    if (DEBUG_SENSORS) {
        float right = SENSORS_distances_average(US_RIGHT);
        int right_int = (int) right;
        snprintf(buffer, sizeof(buffer), "Right = %d\r\t", right_int);
        UART_send_string(buffer);
        
        float left = SENSORS_distances_average(US_LEFT);
        int left_int = (int) left;
        snprintf(buffer, sizeof(buffer), "Left = %d\r\t", left_int);
        UART_send_string(buffer);
        
        UART_send_string("\n");
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
    
    if (DEBUG_LIFT) {
        FAN_set_spin(FAN_LIFT, FAN_MAX);
    }
    
    if (DEBUG_SERVO_SPIN) {
        SERVO_change_angle(ANGLE_SERVO_RIGHT);
        UART_send_string("Right");
        _delay_ms(2000);
        SERVO_change_angle(ANGLE_SERVO_CENTER);
        UART_send_string("Center");
        _delay_ms(2000);
        SERVO_change_angle(ANGLE_SERVO_LEFT);
        UART_send_string("Left");
        _delay_ms(2000);
    }
    
    if (DEBUG_SKIRT) {
        SERVO_change_angle(ANGLE_SERVO_CENTER);
        FAN_set_spin(FAN_LIFT, FAN_MAX);
        FAN_set_spin(FAN_STEER, FAN_MAX);
    }
}
