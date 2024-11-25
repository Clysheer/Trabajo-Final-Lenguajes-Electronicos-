#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_VL6180X.h>
 
// Parámetros del PID
#define KP 4.0         // Ganancia proporcional
#define KD 0.1          // Ganancia derivativa
#define BASE_SPEED 127  // Velocidad base de los motores (0 - 255)
#define FREQ 1000
#define RES 8
#define MAX 90
#define MIN 0

#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 27
#define PIN_MOTOR_B1 13
#define PIN_MOTOR_B2 14

#define PIN_BOTON_START 18

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

#define SHT_LOX1 19 // medio
#define SHT_LOX2 5 //izquierda
#define SHT_LOX3 2 //derecha

#define BEGIN 115200
#define R180 180
#define R90  90
#define GZN  131.0
#define DTN  1000.0
#define SGM 2
#define DM 3
#define LM 5
#define SL 12 // Sensor de piso
#define CAL 2000

#define DMAXP 55 
#define DMAXD 85
#define DMAXI 80
#define DMAXM 100
#define DISTANCE1 0 
#define DISTANCE2 1 //
#define DISTANCE3 2 //izquierda
#define SCL 22
#define SDA 21

MPU6050 sensor; //inicianilizacion sensor 

struct RobotState {
  // Variables del giroscopio
  int16_t gz, gy, gx;
  float gz_cal = 0;
  long tiempo_prev, dt;
  float elapsedTime;
  float girosc_ang_z, girosc_ang_z_prev;
  float angleZ = 0;
  float previousError = 0;

  // Arreglo y puntero para las distancias
  int16_t distance[DM] = {DISTANCE1, DISTANCE2, DISTANCE3};
  int16_t* dr = distance;
};


  Adafruit_VL6180X lox1 = Adafruit_VL6180X();
  Adafruit_VL6180X lox2 = Adafruit_VL6180X();
  Adafruit_VL6180X lox3 = Adafruit_VL6180X();

  
// Función de reposo
void reposo() {
  bool boton_presionado = false;
  while (!boton_presionado) {
    if (digitalRead(PIN_BOTON_START) == LOW) {
      boton_presionado = true;
      delay(200);  // Anti-rebote
    }
  }
}

// Configurar los ID de los sensores
void setID() {
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

 if (!lox1.begin()) while (1);
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox2.begin()) while (1);
  lox2.setAddress(LOX2_ADDRESS);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10); 

  if (!lox3.begin()) while (1);
  lox3.setAddress(LOX3_ADDRESS);
}

void start_sensors() {
  sensor.initialize();  // Inicializa el MPU6050
  pinMode(PIN_BOTON_START, INPUT_PULLUP);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  setID();
  calibrateMPU6050();
  tiempo_prev = millis();
  // Configuración de los pines de los motores
  ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
  ledcSetup(CHANNEL1, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
  ledcSetup(CHANNEL2, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
  ledcSetup(CHANNEL3, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);
  ledcSetup(CHANNEL4, FREQ, RES);
}

void readDistance(Adafruit_VL6180X &vl, uint16_t &distance) {
  uint16_t range = vl.readRange();
  distance = range;
}

void measurement(RobotState& state) {
  uint16_t distance1, distance2, distance3;
  readDistance(lox1, distance1);
  state.dr[0] = distance1;
  readDistance(lox2, distance2);
  state.dr[1] = distance2;
  readDistance(lox3, distance3);
  state.dr[2] = distance3;
}


void moveForward() {
  ledcWrite(CHANNEL1, MAX);  
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MAX);  
  ledcWrite(CHANNEL4, MIN);
}

void stopMotors() {
  ledcWrite(CHANNEL1, MIN);
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);
  ledcWrite(CHANNEL4, MIN);
}
void moveLeft() {
  ledcWrite(CHANNEL1, MAX); 
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);   
  ledcWrite(CHANNEL4, MAX);
}

void moveRight() {
  ledcWrite(CHANNEL1, MIN);   
  ledcWrite(CHANNEL2, MAX);
  ledcWrite(CHANNEL3, MAX);  
  ledcWrite(CHANNEL4, MIN);
}

// Función para girar a la derecha
void rotateRight() {
  ledcWrite(CHANNEL1, MIN); 
  ledcWrite(CHANNEL2, MAX);
  ledcWrite(CHANNEL3, MAX );   
  ledcWrite(CHANNEL4, MIN);
}

// Función para girar a la izquierda
void rotateLeft() {
  ledcWrite(CHANNEL1, MAX); 
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);   
  ledcWrite(CHANNEL4, MAX);
}

// PID para el sensor VL6180X
void pidControl() {
  float error, derivative, pidOutput;
  error = DMAXP - distance[DISTANCE2];
  derivative = error - previousError;
  pidOutput = (KP * error) + (KD * derivative);
  previousError = error;

  int motorSpeedLeft = constrain(BASE_SPEED + pidOutput, MIN, MAX);
  int motorSpeedRight = constrain(BASE_SPEED - pidOutput, MIN, MAX);

  controlMotor(motorSpeedLeft, motorSpeedRight);
  delay(100);
}

void controlMotor(int speedLeft, int speedRight) {
  if (speedLeft > 0) {
    ledcWrite(CHANNEL1, speedLeft);  
    ledcWrite(CHANNEL2, MIN);       
  } else {
    ledcWrite(CHANNEL1, MIN);       
    ledcWrite(CHANNEL2, abs(speedLeft));  
  }

  if (speedRight > 0) {
    ledcWrite(CHANNEL3, speedRight ); 
    ledcWrite(CHANNEL4, MIN);       
  } else {
    ledcWrite(CHANNEL3, MIN);       
    ledcWrite(CHANNEL4, abs(speedRight));  
  }
}

// Función para rotar hacia la derecha 
void RRight(float degrees, RobotState& state) {
  float targetAngle;  
  sensor.getRotation(&state.gx, &state.gy, &state.gz);
  state.angleZ += (state.gz - state.gz_cal) * (1.0 / GZN);
  targetAngle = state.angleZ + degrees;
  rotateRight();

  while (true) {
    sensor.getRotation(&state.gx, &state.gy, &state.gz);
    state.angleZ += (state.gz - state.gz_cal) * (1.0 / GZN);
    if ((degrees > 0 && state.angleZ >= targetAngle) || (degrees < 0 && state.angleZ <= targetAngle)) {
      stopMotors();
      break;
    }
  }
}

// Función para rotar hacia la izquierda
void RLeft(float degrees, RobotState& state) {
  float targetAngle;  
  sensor.getRotation(&state.gx, &state.gy, &state.gz);
  state.angleZ += (state.gz - state.gz_cal) * (1.0 / GZN);
  targetAngle = state.angleZ + degrees;
  rotateLeft();

  while (true) {
    sensor.getRotation(&state.gx, &state.gy, &state.gz);
    state.angleZ += (state.gz - state.gz_cal) * (1.0 / GZN);
    if ((degrees > 0 && state.angleZ >= targetAngle) || (degrees < 0 && state.angleZ <= targetAngle)) {
      stopMotors();
      break;
    }
  }
}

void calibrateMPU6050(RobotState& state) {
  const int calibrations = CAL;
  float sumGz = 0;
  for (int i = 0; i < calibrations; i++) {
    sensor.getRotation(&state.gx, &state.gy, &state.gz);
    sumGz += state.gz;
    delay(3);
  }
  state.gz_cal = sumGz / calibrations;
}

enum estadosMovimiento
{
  PISODETECTADO,
  MOVEFORWARD,
  ROTATEL90,
  ROTATER90,
  ROTATE180
};

void movements() { 
    int8_t piso = digitalRead(SL);
    estadosMovimiento state; 
    measurement(distance);
    if (piso == HIGH){  
        state = PISODETECTADO;  //detecta piso
    }
    else if (distance[DISTANCE3] > DMAXI && distance[DISTANCE1] > DMAXM && distance[DISTANCE2] < DMAXD  ) { 
    //detecta derecha
        state =  MOVEFORWARD ;   
    }
    else if (distance[DISTANCE3] > DMAXI && distance[DISTANCE1] < DMAXM && distance[DISTANCE2] < DMAXD ) {  
    //detectan medio y derecha 
        state = ROTATEL90 ;  //rotatel90 
    }
   else if (distance[DISTANCE3] < DMAXI && distance[DISTANCE1] > DMAXM && distance[DISTANCE2] < DMAXD ) { 
    //detectan izquierda y derecha  
        state =  MOVEFORWARD;  
    }
    else if (distance[DISTANCE3] < DMAXI && distance[DISTANCE1] < DMAXM && distance[DISTANCE2] > DMAXD ) { 
     //detectan medio y izquierda 
        state = ROTATER90;  //ROTATER90  
    }
    else if (distance[DISTANCE3] < DMAXI && distance[DISTANCE1] > DMAXM && distance[DISTANCE2] > DMAXD ) {
     //detectan izquierda 
        state = ROTATER90 ;   //ROTATER90 
    }
    else if (distance[DISTANCE3] > DMAXI && distance[DISTANCE1] < DMAXM && distance[DISTANCE2] > DMAXD ) { 
    //detecta sensor distancia medio 
        state = ROTATEL90;   
    }
    else if (distance[DISTANCE3] < DMAXI && distance[DISTANCE1] < DMAXM && distance[DISTANCE2] < DMAXD) { 
    //detectan todos los sensores de distancia 
        state = ROTATEL90;  
    }
    else if (distance[DISTANCE3] > DMAXI && distance[DISTANCE1] > DMAXM && distance[DISTANCE2] > DMAXD ) {  
    // no detecta ninguno sensor de distancia 
        state = ROTATER90;   
    }

   switch (state) {
        case PISODETECTADO: 
            stopMotors(); 
            break; 
        case MOVEFORWARD: 
            pidControl();
            break;
        case ROTATEL90:  
            RLeft(R90);
            break;
        case ROTATER90:  
            RRight(-R90);
            break;
        case ROTATE180: 
            RLeft(R180);
            break; 
    }
} 

void start() {
  Serial.begin(BEGIN);
  Wire.begin();
}

void setup() {
  RobotState state; 
  start();
  start_sensors();
  reposo();
}

void loop() {
  RobotState state; 
  movements();
}
