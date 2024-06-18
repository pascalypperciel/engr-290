#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <Wire.h>

// MPU6050_light.h content starts here
#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b

#define MPU6050_GYRO_OUT_REGISTER     0x43
#define MPU6050_ACCEL_OUT_REGISTER    0x3B

#define RAD_2_DEG             57.29578 // [deg/rad]
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0

#define DEFAULT_GYRO_COEFF    0.98

class MPU6050 {
  public:
    MPU6050(TwoWire &w);
    byte begin(int gyro_config_num=1, int acc_config_num=0);

    byte writeData(byte reg, byte data);
    byte readData(byte reg);

    void calcOffsets(bool is_calc_gyro=true, bool is_calc_acc=true);
    void calcGyroOffsets(){ calcOffsets(true,false); };
    void calcAccOffsets(){ calcOffsets(false,true); };

    void setAddress(uint8_t addr){ address = addr; };
    uint8_t getAddress(){ return address; };

    byte setGyroConfig(int config_num);
    byte setAccConfig(int config_num);

    void setGyroOffsets(float x, float y, float z);
    void setAccOffsets(float x, float y, float z);

    void setFilterGyroCoef(float gyro_coeff);
    void setFilterAccCoef(float acc_coeff);

    float getGyroXoffset(){ return gyroXoffset; };
    float getGyroYoffset(){ return gyroYoffset; };
    float getGyroZoffset(){ return gyroZoffset; };

    float getAccXoffset(){ return accXoffset; };
    float getAccYoffset(){ return accYoffset; };
    float getAccZoffset(){ return accZoffset; };

    float getFilterGyroCoef(){ return filterGyroCoef; };
    float getFilterAccCoef(){ return 1.0-filterGyroCoef; };

    float getTemp(){ return temp; };

    float getAccX(){ return accX; };
    float getAccY(){ return accY; };
    float getAccZ(){ return accZ; };

    float getGyroX(){ return gyroX; };
    float getGyroY(){ return gyroY; };
    float getGyroZ(){ return gyroZ; };

    float getAccAngleX(){ return angleAccX; };
    float getAccAngleY(){ return angleAccY; };

    float getAngleX(){ return angleX; };
    float getAngleY(){ return angleY; };
    float getAngleZ(){ return angleZ; };

    void fetchData();
    void update();

    bool upsideDownMounting = false;

  private:
    TwoWire *wire;
    uint8_t address = MPU6050_ADDR; // 0x68 or 0x69
    float gyro_lsb_to_degsec, acc_lsb_to_g;
    float gyroXoffset, gyroYoffset, gyroZoffset;
    float accXoffset, accYoffset, accZoffset;
    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
    float angleAccX, angleAccY;
    float angleX, angleY, angleZ;
    long preInterval;
    float filterGyroCoef; // complementary filter coefficient to balance gyro vs accelero data to get angle
};
// MPU6050_light.h content ends here

// MPU6050_light.cpp content starts here
MPU6050::MPU6050(TwoWire &w) {
  wire = &w;
  gyro_lsb_to_degsec = 131.0;
  acc_lsb_to_g = 16384.0;
  filterGyroCoef = DEFAULT_GYRO_COEFF;
  preInterval = 0;
  angleX = 0;
  angleY = 0;
  angleZ = 0;
}

byte MPU6050::begin(int gyro_config_num, int acc_config_num) {
  wire->begin();
  if (writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x00) != 0) return 1; // exit sleep mode

  if (setGyroConfig(gyro_config_num) != 0) return 2;
  if (setAccConfig(acc_config_num) != 0) return 3;

  return 0;
}

byte MPU6050::writeData(byte reg, byte data) {
  wire->beginTransmission(address);
  wire->write(reg);
  wire->write(data);
  return wire->endTransmission();
}

byte MPU6050::readData(byte reg) {
  wire->beginTransmission(address);
  wire->write(reg);
  if (wire->endTransmission(false) != 0) return 0;
  wire->requestFrom(address, (byte)1);
  return wire->read();
}

void MPU6050::calcOffsets(bool is_calc_gyro, bool is_calc_acc) {
  float gx, gy, gz, ax, ay, az;
  gx = gy = gz = ax = ay = az = 0;

  for (int i = 0; i < CALIB_OFFSET_NB_MES; i++) {
    fetchData();

    if (is_calc_gyro) {
      gx += gyroX;
      gy += gyroY;
      gz += gyroZ;
    }
    if (is_calc_acc) {
      ax += accX;
      ay += accY;
      az += accZ - acc_lsb_to_g; // take gravity into account
    }
    delay(5);
  }

  if (is_calc_gyro) {
    gyroXoffset = gx / CALIB_OFFSET_NB_MES;
    gyroYoffset = gy / CALIB_OFFSET_NB_MES;
    gyroZoffset = gz / CALIB_OFFSET_NB_MES;
  }
  if (is_calc_acc) {
    accXoffset = ax / CALIB_OFFSET_NB_MES;
    accYoffset = ay / CALIB_OFFSET_NB_MES;
    accZoffset = az / CALIB_OFFSET_NB_MES;
  }
}

byte MPU6050::setGyroConfig(int config_num) {
  switch (config_num) {
    case 0: gyro_lsb_to_degsec = 131.0; break;
    case 1: gyro_lsb_to_degsec = 65.5; break;
    case 2: gyro_lsb_to_degsec = 32.8; break;
    case 3: gyro_lsb_to_degsec = 16.4; break;
    default: return 1;
  }
  return writeData(MPU6050_GYRO_CONFIG_REGISTER, config_num << 3);
}

byte MPU6050::setAccConfig(int config_num) {
  switch (config_num) {
    case 0: acc_lsb_to_g = 16384.0; break;
    case 1: acc_lsb_to_g = 8192.0; break;
    case 2: acc_lsb_to_g = 4096.0; break;
    case 3: acc_lsb_to_g = 2048.0; break;
    default: return 1;
  }
  return writeData(MPU6050_ACCEL_CONFIG_REGISTER, config_num << 3);
}

void MPU6050::fetchData() {
  wire->beginTransmission(address);
  wire->write(MPU6050_ACCEL_OUT_REGISTER);
  wire->endTransmission(false);
  wire->requestFrom(address, (byte)14);

  accX = (int16_t)(wire->read() << 8 | wire->read()) / acc_lsb_to_g;
  accY = (int16_t)(wire->read() << 8 | wire->read()) / acc_lsb_to_g;
  accZ = (int16_t)(wire->read() << 8 | wire->read()) / acc_lsb_to_g;
  temp = ((int16_t)(wire->read() << 8 | wire->read()) + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((int16_t)(wire->read() << 8 | wire->read()) / gyro_lsb_to_degsec) - gyroXoffset;
  gyroY = ((int16_t)(wire->read() << 8 | wire->read()) / gyro_lsb_to_degsec) - gyroYoffset;
  gyroZ = ((int16_t)(wire->read() << 8 | wire->read()) / gyro_lsb_to_degsec) - gyroZoffset;

  if (upsideDownMounting) {
    accX = -accX;
    accY = -accY;
    accZ = -accZ;
    gyroX = -gyroX;
    gyroY = -gyroY;
    gyroZ = -gyroZ;
  }

  angleAccX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * RAD_2_DEG;
  angleAccY = atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * RAD_2_DEG;
}

void MPU6050::update() {
  fetchData();

  long interval = millis() - preInterval;
  if (interval > 0) {
    angleX = filterGyroCoef * (angleX + gyroX * interval * 0.001) + (1.0 - filterGyroCoef) * angleAccX;
    angleY = filterGyroCoef * (angleY + gyroY * interval * 0.001) + (1.0 - filterGyroCoef) * angleAccY;
    angleZ += gyroZ * interval * 0.001;

    preInterval = millis();
  }
}

void MPU6050::setGyroOffsets(float x, float y, float z) {
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::setAccOffsets(float x, float y, float z) {
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void MPU6050::setFilterGyroCoef(float gyro_coeff) {
  filterGyroCoef = gyro_coeff;
}

void MPU6050::setFilterAccCoef(float acc_coeff) {
  filterGyroCoef = 1.0 - acc_coeff;
}
// MPU6050_light.cpp content ends here


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
MPU6050           mpu(Wire);
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
      float yaw = mpu.getAngleZ();
      return yaw;
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
}

void GENERAL_turn(float opening_angle) {
    MPU_change_target_yaw();
    SERVO_change_angle(opening_angle);
    while (!MPU_is_turn_over()) {
        mpu.update();
    }
}

void forward_logic() {
  //this code needs to take 90 (away) or 270 (towards) yaw and orient towards that, ideally the servo stays near 90. This will run until opening is detected.
  float imu_error;
  if (target_yaw == ANGLE_YAW_AWAY) {
    imu_error = target_yaw - MPU_get_yaws();
  }
  else if (target_yaw == ANGLE_YAW_TOWARDS) {
    imu_error = target_yaw - MPU_get_yaws() - 180;
  }
  if(imu_error > 180) { imu_error = 180; }
  if(imu_error < 0) { imu_error = 0; }
  SERVO_change_angle(imu_error);
  mpu.update();
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
    mpu.calcOffsets();
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
