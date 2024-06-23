#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <util/twi.h>

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

#define MILLIS_INC          1
#define FRACT_INC           3
#define FRACT_MAX           1000

#define MPU6050_ADDRESS     0x68
#define TWI_BUFFER_LENGTH   32

#define   TW_START   0x08
#define   TW_REP_START   0x10
#define   TW_MT_SLA_ACK   0x18
#define   TW_MT_SLA_NACK   0x20
#define   TW_MT_DATA_ACK   0x28
#define   TW_MT_DATA_NACK   0x30
#define   TW_MT_ARB_LOST   0x38
#define   TW_MR_ARB_LOST   0x38
#define   TW_MR_SLA_ACK   0x40
#define   TW_MR_SLA_NACK   0x48
#define   TW_MR_DATA_ACK   0x50
#define   TW_MR_DATA_NACK   0x58
#define   TW_ST_SLA_ACK   0xA8
#define   TW_ST_ARB_LOST_SLA_ACK   0xB0
#define   TW_ST_DATA_ACK   0xB8
#define   TW_ST_DATA_NACK   0xC0
#define   TW_ST_LAST_DATA   0xC8
#define   TW_SR_SLA_ACK   0x60
#define   TW_SR_ARB_LOST_SLA_ACK   0x68
#define   TW_SR_GCALL_ACK   0x70
#define   TW_SR_ARB_LOST_GCALL_ACK   0x78
#define   TW_SR_DATA_ACK   0x80
#define   TW_SR_DATA_NACK   0x88
#define   TW_SR_GCALL_DATA_ACK   0x90
#define   TW_SR_GCALL_DATA_NACK   0x98
#define   TW_SR_STOP   0xA0
#define   TW_NO_INFO   0xF8
#define   TW_BUS_ERROR   0x00
#define   TW_READ   1 
#define   TW_WRITE   0
#define   TWI_READY 0
#define   TWI_MRX   1
#define   TWI_MTX   2
#define   TWI_SRX   3
#define   TWI_STX   4

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
  int16_t X;
  int16_t Y;
  int16_t Z;
} Gyroscope;

typedef struct {
  float ROLL;
  float PITCH;
  float YAW;
  float ANGLE_X;
  float ANGLE_Y;
  float ANGLE_Z;
  uint32_t TIME;
  int ERRORS;
} Directions;

typedef struct {
  uint8_t SDA_PIN;
  uint8_t SCL_PIN;
  int ADDRESS;
  Gyroscope GYRO;
  Directions DIRECTIONS;
} InertialMeasurementUnit;

typedef struct {
  float GYRO_X;
  float GYRO_Y;
  float GYRO_Z;
} Calibration;

// Components
UltrasonicSensor        US_RIGHT =    {PD2, PB3};
UltrasonicSensor        US_LEFT =     {PD3, PB5};
ServoMotor              SERVO =       {PB1};
Fan                     FAN_LIFT =    {PD5};
Fan                     FAN_STEER =   {PD6};
InertialMeasurementUnit IMU =         {PC4, PC5, 0x68, {0, 0, 0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0}};

//Others
Calibration             CALIBRATION = {0, 0, 0};

// Global variable
int _target_yaw = ANGLE_YAW_AWAY;
int _opening_in_a_row_right = 0;
int _opening_in_a_row_left = 0;
bool _last_was_big_right = false;
bool _last_was_big_left = false;
float _d_last = 0;
volatile unsigned long _timer0_millis = 0;
volatile unsigned char _timer0_fract = 0;
static unsigned long _timer0_overflow_count = 0;
uint8_t _IMU_transmitting = 1;
uint8_t _IMU_transmit_buffer_index = 0;
uint8_t _IMU_transmit_buffer_length = 0;
uint8_t _IMU_transmit_buffer[TWI_BUFFER_LENGTH];
uint8_t _IMU_receive_buffer[TWI_BUFFER_LENGTH];
uint8_t _IMU_receive_buffer_index = 0;
uint8_t _IMU_receive_buffer_length = 0;
uint8_t _TWI_transmit_buffer_length = 0;
uint8_t _TWI_transmit_buffer[TWI_BUFFER_LENGTH];
static volatile uint8_t _TWI_state;
static volatile uint8_t _TWI_error;
uint8_t _TWI_send_stop = true;
uint8_t _TWI_master_buffer_index = 0;
uint8_t _TWI_master_buffer_length = 0;
uint8_t _TWI_master_buffer[TWI_BUFFER_LENGTH];
uint8_t _TWI_in_rep_start = false;
static volatile uint8_t _TWI_slarw;
uint8_t _TWI_timeout_us;
bool _TWI_timed_out_flag;

// Function Declarations
void UART_init(void);
void UART_send_char(unsigned char data);
void UART_send_string(const char *str);
float SENSORS_measure_distance(UltrasonicSensor US);
int SENSORS_opening_detected(void);
void SERVO_init_timer1(void);
void SERVO_change_angle(float angle);
// void FAN_init(void);
// void FAN_set_spin(Fan fan, int value);
void MPU_change_target_yaw(void);
bool MPU_is_turn_over(void);
float MPU_get_yaw(void);
float MPU_get_yaw_turning(void);
bool IMU_init(void);
float IMU_get_yaw(void);
float IMU_read_gyroscope_yaw(void);
void IMU_update_yaw(void);
int IMU_absent(void);
bool IMU_wake(void);
bool IMU_set_rate_divider(void);
int IMU_set_DLPF_bandwidth(void);
bool IMU_calibrate(void);
int IMU_read_8(uint8_t addr, uint8_t *value);
bool IMU_read_3x16(uint8_t addr, uint16_t *value0, uint16_t *value1, uint16_t *value2);
bool IMU_write_8(uint8_t addr, uint8_t value);
unsigned long GENERAL_millis(void);
void GENERAL_init_timer0(void);
void GENERAL_init_interrupts(void);
void GENERAL_init_ports(void);
void GENERAL_components_setup(void);
void GENERAL_go_forward(void);
void GENERAL_make_turn(float opening_angle);

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

  OCR1A = (angle - 0) * (4300 - 800) / (180 - 0) + 800;
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

void TWI_init() {
  TWSR &= ~((1 << TWPS0) | (1 << TWPS1));
  TWBR = ((F_CPU / 100000L) - 16) / 2;
  TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
  _TWI_state = TWI_READY;
  _TWI_send_stop = true;
  _TWI_in_rep_start = false;
  _TWI_timed_out_flag = false;
}

uint8_t TWI_transmit(const uint8_t* data, uint8_t length) {
  uint8_t i;

  if (TWI_BUFFER_LENGTH < (_TWI_transmit_buffer_length + length)) {
    return 1;
  }

  for (int i = 0; i < length; ++i) {
    _TWI_transmit_buffer[_TWI_transmit_buffer_length + i] = data[i];
  }
  _TWI_transmit_buffer_length += length;

  return 0;
}

void TWI_handle_timeout(bool reset) {
  _TWI_timed_out_flag = true;

  if (reset) {
    //todo
  }
}

uint8_t TWI_write(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t send_stop) {
  uint8_t i;

  if (TWI_BUFFER_LENGTH < length) {
    return 1;
  }

  uint32_t start_micros = GENERAL_micros();
//  while (_TWI_state != 0) {
//    if ((GENERAL_micros() - start_micros) > 0ul) {
//      TWI_handle_timeout(false);
//      return (5);
//    }
//  }
  _TWI_state = 2;
  _TWI_send_stop = send_stop;
  _TWI_error = 0xFF;

  _TWI_master_buffer_index = 0;
  _TWI_master_buffer_length = length;

  for (int i = 0; i < length; ++i) {
    _TWI_master_buffer[i] = data[i];
  }

  _TWI_slarw = 0;
  _TWI_slarw |= address << 1;

  if (_TWI_in_rep_start) {
    _TWI_in_rep_start = false;
    start_micros = GENERAL_micros();
    do {
      TWDR = _TWI_slarw;
//      if ((GENERAL_micros() - start_micros) > 0ul) {
//        TWI_handle_timeout(false);
//        return (5);
//      }
    } while (TWCR & (1 << TWWC));
    TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
  } else {
    TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTA);
  }

  return 0;
}

uint8_t TWI_read(uint8_t address, uint8_t* data, uint8_t length, uint8_t send_stop) {
  uint8_t i;

  if (TWI_BUFFER_LENGTH < length) {
    return 0;
  }
  
  uint32_t start_micros = GENERAL_micros();
  while (_TWI_state != TWI_READY) {
    if ((GENERAL_micros() - start_micros) > 10000) {
      TWI_handle_timeout(false);
      return 69421;
    }
  }

  _TWI_state = TWI_MRX;
  _TWI_send_stop = send_stop;
  _TWI_error = 0xFF;

  _TWI_master_buffer_index = 0;
  _TWI_master_buffer_length = length - 1;
  
  _TWI_slarw = 1;
  _TWI_slarw |= address << 1;

  if (_TWI_in_rep_start) {
    _TWI_in_rep_start = false;
    start_micros = GENERAL_micros();
    do {
      TWDR = _TWI_slarw;
    } while (TWCR & (1 << TWWC));
    TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
  } else {
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT) | (1 << TWSTA);
  }

  start_micros = GENERAL_micros();
  while (_TWI_state == TWI_MRX) {
    if ((GENERAL_micros() - start_micros) > 10000) {
      TWI_handle_timeout(false);
      return 69421;
    }
  }

  if (_TWI_master_buffer_index < length) {
    length = _TWI_master_buffer_index;
  }

  for (int i = 0; i < length; ++i) {
    data[i] = _TWI_master_buffer[i];
  }

  return length;
}

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
  float yaw = IMU_get_yaw();
  char buffer[16];
  dtostrf((int)yaw, 6, 2, buffer);
  UART_send_string("\nYaw : \t");
  UART_send_string(buffer);
  return yaw;
}

float MPU_get_yaw_turning() {
  char buffer[32];
  char _target_yaw_buffer[16];
  
  float yaw = IMU_get_yaw();
  
  dtostrf((int)yaw, 6, 2, buffer);
  dtostrf(_target_yaw, 6, 2, _target_yaw_buffer);
  
  strcat(buffer, " _target_yaw: ");
  strcat(buffer, _target_yaw_buffer);
  
  UART_send_string("\nYee: \t");
  UART_send_string(buffer);
  
  return yaw;
}

bool IMU_init() {
  char buffer[32];
  int error = IMU_absent();
  if (error != 0) {
    snprintf(buffer, sizeof(buffer), "\nTESTING1, Error: %d", error);
    UART_send_string(buffer);
    return false;
  }
  if (IMU_wake()) {
    UART_send_string("\nTESTING2");
    return false;
  }
  if (IMU_set_rate_divider()) {
    UART_send_string("\nTESTING3");
    return false;
  }
  error = IMU_set_DLPF_bandwidth();
  if (error != 0) {
    snprintf(buffer, sizeof(buffer), "\nTESTING4, Error: %d", error);
    UART_send_string(buffer);
    return false;
  }
  if (IMU_calibrate()) {
    UART_send_string("\nTESTING5");
    return false;
  }

  IMU.DIRECTIONS.TIME = GENERAL_millis();

  return true;
}

float IMU_get_yaw() {
  IMU_update_yaw();
  return IMU.DIRECTIONS.YAW;
}

float IMU_read_gyroscope_yaw() {
  int16_t gyro_raw_z;
  IMU_read_3x16(0x43, NULL, NULL, &gyro_raw_z);
  return gyro_raw_z * (1.0/131.0) - CALIBRATION.GYRO_Z;
}

void IMU_update_yaw() {
  float gyro_z = IMU_read_gyroscope_yaw();
  uint32_t now = GENERAL_millis();
  float time_difference = (now - IMU.DIRECTIONS.TIME) / 1000.0;
  IMU.DIRECTIONS.TIME = now;
  IMU.DIRECTIONS.ANGLE_Z += gyro_z * time_difference;
  IMU.DIRECTIONS.YAW = 1.00 * IMU.DIRECTIONS.ANGLE_Z;
}

int IMU_absent() {
    uint8_t val;
    int error= IMU_read_8(0x75,&val);
    if( error != 0 ) return error;
    if( val!= 0x68 ) return 69420;
    return 0;
}

bool IMU_wake() {
  return IMU_write_8(0x6B, 0b00000000);
}

bool IMU_set_rate_divider() {
  return IMU_write_8(0x19, 6);
}

int IMU_set_DLPF_bandwidth() {
  uint8_t val;
  if (IMU_read_8(0x1A, &val)) return -1;
  val = (val & 0b11111000) | 0;
  return IMU_write_8(0x1A, val);
}

bool IMU_calibrate() {
  Calibration cal;
  CALIBRATION.GYRO_Z = 0;
  cal.GYRO_Z = 0;

  for (int i = 0; i < 500; i++) {
    cal.GYRO_Z += IMU_read_gyroscope_yaw();
  }
  
  CALIBRATION.GYRO_Z = cal.GYRO_Z / 500;
  return false;
}

void IMU_begin_transmission() {
  _IMU_transmitting = 1;
  _IMU_transmit_buffer_index = 0;
  _IMU_transmit_buffer_length = 0;
}

int IMU_write(uint8_t data) {
  if (_IMU_transmitting) {
    if (_IMU_transmit_buffer_length >= 32) {
       return 0;
    }
    _IMU_transmit_buffer[_IMU_transmit_buffer_index] = data;
    ++_IMU_transmit_buffer_index;
    _IMU_transmit_buffer_length = _IMU_transmit_buffer_index;
  } else {
    TWI_transmit(&data, 1);
  }

  return 1;
}

uint8_t IMU_end_transmission(uint8_t send_stop) {
  uint8_t ret = TWI_write(MPU6050_ADDRESS, _IMU_transmit_buffer, _IMU_transmit_buffer_length, 1, send_stop);
  _IMU_transmit_buffer_index = 0;
  _IMU_transmit_buffer_length = 0;
  _IMU_transmitting = 0;
  return ret;
}

uint8_t IMU_request_from(uint8_t address, uint8_t quantity, uint8_t send_stop) {
  if (quantity > TWI_BUFFER_LENGTH) {
    quantity = TWI_BUFFER_LENGTH;
  }

  uint8_t read = TWI_read(address, _IMU_receive_buffer, quantity, send_stop);
  _IMU_receive_buffer_index = 0;
  _IMU_receive_buffer_length = read;

  return read;
}

int IMU_read() {
  int value = -1;
  if (_IMU_receive_buffer_index < _IMU_receive_buffer_length) {
    value = _IMU_receive_buffer[_IMU_receive_buffer_index];
    ++_IMU_receive_buffer_index;
  }

  return value;
}

int IMU_read_8(uint8_t addr, uint8_t *value) {
    *value=0;
    IMU_begin_transmission();
    int r1 = IMU_write(addr);
    if( r1 != 1 ) return -1;
    int r2 = IMU_end_transmission(false);                            
    if( r2 != 0 ) return -2;
    int r3 = IMU_request_from(MPU6050_ADDRESS,(uint8_t)1,(uint8_t)true);
    if( r3 != 1 ) return r3;
    *value = IMU_read();
    return 0;
}

bool IMU_read_3x16(uint8_t addr, uint16_t *value0,  uint16_t *value1, uint16_t *value2 ) {
    *value0= *value1= *value2= 0;
    IMU_begin_transmission();
    int r1 = IMU_write(addr);
    if( r1!=1 ) return true;
    int r2 = IMU_end_transmission(false);
    if( r2!=0 ) return true;
    int r3 = IMU_request_from(MPU6050_ADDRESS,(uint8_t)6,(uint8_t)true);
    if( r3!=6 ) return true;
    *value0 = IMU_read() << 8 | IMU_read(); 
    *value1 = IMU_read() << 8 | IMU_read(); 
    *value2 = IMU_read() << 8 | IMU_read(); 
    return false;
}

bool IMU_write_8(uint8_t addr, uint8_t value) {
    IMU_begin_transmission();
    int r1 = IMU_write(addr);
    if( r1!=1 ) return 10;
    int r2 = IMU_write(value);
    if( r2!=1 ) return 10;
    int r3 = IMU_end_transmission(true);
    if( r3!=0 ) return r3;
    return 0;
}

//// Read 'value' from the registers starting at 'addr'. Returns error; see error_str() for explanation.
//bool IMU_read_8(uint8_t addr, uint8_t *value) {
//    *value=0;
//    Wire.beginTransmission(0x68);
//    int r1=Wire.write(addr);                                       if( r1!=1 ) return 10;
//    int r2=Wire.endTransmission(false);                            if( r2!=0 ) return 2;
//    int r3=Wire.requestFrom(0x68,(uint8_t)1,(uint8_t)true); if( r3!=1 ) return 11;
//    *value=Wire.read();
//    return 0;
//}
//
//// Read 3x'value' from the 3 registers starting at 'addr'. Returns error; see error_str() for explanation.
//bool IMU_read_3x16(uint8_t addr, uint16_t *value0,  uint16_t *value1, uint16_t *value2 ) {
//    *value0= *value1= *value2= 0;
//    Wire.beginTransmission(0x68);
//    int r1=Wire.write(addr);                                       if( r1!=1 ) return 10;
//    int r2=Wire.endTransmission(false);                            if( r2!=0 ) return r2;
//    int r3=Wire.requestFrom(0x68,(uint8_t)6,(uint8_t)true); if( r3!=6 ) return 11;
//    *value0=Wire.read()<<8 | Wire.read(); 
//    *value1=Wire.read()<<8 | Wire.read(); 
//    *value2=Wire.read()<<8 | Wire.read(); 
//    return 0;
//}
//
//// Write 'value' to register at address 'addr'. Returns error; see error_str() for explanation.
//bool IMU_write_8(uint8_t addr, uint8_t value) {
//    Wire.beginTransmission(0x68);
//    int r1=Wire.write(addr);                                       if( r1!=1 ) return 10;
//    int r2=Wire.write(value);                                      if( r2!=1 ) return 10;
//    int r3=Wire.endTransmission(true);                             if( r3!=0 ) return r3;
//    return 0;
//}

unsigned long GENERAL_millis() {
  unsigned long m;
  uint8_t oldSREG = SREG;
  
  cli();
  m = _timer0_millis;
  SREG = oldSREG;

  return m;
}

unsigned long GENERAL_micros() {
  unsigned long m;
  uint8_t oldSREG = SREG, t;
  
  cli();
  m = _timer0_overflow_count;
#if defined(TCNT0)
  t = TCNT0;
#elif defined(TCNT0L)
  t = TCNT0L;
#else
  #error TIMER 0 not defined
#endif

#ifdef TIFR0
  if ((TIFR0 & _BV(TOV0)) && (t < 255))
    m++;
#else
  if ((TIFR & _BV(TOV0)) && (t < 255))
    m++;
#endif

  SREG = oldSREG;
  
  return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
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
    imu_error = 90 - (_target_yaw + yaw);
    PID_constant = _target_yaw + yaw;
    if (abs(PID_constant) > 5) {
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
  TWI_init();
  GENERAL_init_timer0();
  GENERAL_init_interrupts();
  GENERAL_init_ports();
  UART_init();
  SERVO_init_timer1();
  //    FAN_init();
  IMU_init();
  GENERAL_components_setup();
  
  int counter = WAIT_AFTER_TURN;
  _opening_in_a_row_right = 0;
  _opening_in_a_row_left = 0;
  _last_was_big_right = false;
  _last_was_big_left = false;

  while (1) {
    if (counter > 0) {
      counter--;
    }
    GENERAL_go_forward();
    
    int opening_angle = SENSORS_opening_detected();
    if (opening_angle != -1 && counter == 0) {
      GENERAL_make_turn(opening_angle);
      counter = WAIT_AFTER_TURN;
    }
  }
}

ISR(TIMER0_OVF_vect) {
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
//ISR(TWI_vect) {
//  switch (TW_STATUS) {
//    case TW_START:
//    case TW_REP_START:
//      TWDR = _TWI_slarw;
//      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
//      break;
//    case TW_MT_SLA_ACK:
//    case TW_MT_DATA_ACK:
//      if (_TWI_master_buffer_index < _TWI_master_buffer_length) {
//        TWDR = _TWI_master_buffer[_TWI_master_buffer_index++];
//        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
//      } else {
//        if (_TWI_send_stop) {
//          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWSTO);
//        } else {
//          _TWI_in_rep_start = true;
//          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTA);
//        }
//        _TWI_state = 0;
//      }
//      break;
//    case TW_MR_DATA_ACK:
//      _TWI_master_buffer[_TWI_master_buffer_index++] = TWDR;
//    case TW_MR_SLA_ACK:
//      if (_TWI_master_buffer_index < _TWI_master_buffer_length) {
//        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
//      } else {
//        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
//      }
//      break;
//    case TW_MR_DATA_NACK:
//      _TWI_master_buffer[_TWI_master_buffer_index++] = TWDR;
//      if (_TWI_send_stop) {
//        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWSTO);
//      } else {
//        _TWI_in_rep_start = true;
//        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTA);
//      }
//      _TWI_state = 0;
//      break;
//    case TW_MT_SLA_NACK:
//    case TW_MT_DATA_NACK:
//    case TW_MR_SLA_NACK:
//    case TW_BUS_ERROR:
//      _TWI_error = TW_STATUS;
//      if (_TWI_send_stop) {
//        TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
//      }
//      _TWI_state = 0;
//      break;
//  }
//}
