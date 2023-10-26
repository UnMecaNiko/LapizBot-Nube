// Desarrollado por @unmecaniko el 25/10/2023

/*
Aspectos a mejorar

* separar por funciones

Calibration
PID
robotMovement
printData

* Eliminar basura

* Hacer el PID con temporizador

* Mejorar la comunición serial, para ajustar los valores del PID

*************
Este código no tiene en cuenta unidades reales en el control PID ni en el error
una futura versión debería tener en cuenta la distancia entre ruedas, la caracterización
de los motores, la distancia del sensor, etc

*/

//librerias
#include <sky.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>


//SoftwareSerial Serial_1(9, 1);

// llamar librerias

sky Robot;
QTRSensors qtr;

//Seleccionar cantidad de sensores

const uint8_t SensorCount = 6;

//LLamar libreria para almacenar los valores en las variables
uint16_t sensorValues[SensorCount];

//Valores control PID

float KP = 0.75;    // 0.9
float KD = 0.75;    // 4
float KI = 0.0001;  //0.0001

//Variables  motores y error pid
int error = 0;
int lastError = 0;

int linealSpeed = 0;
int angularSpeed;

//Funciones

void printData(float batteryVoltage, float KP, float KD, int error, int angularSpeed);
int pid(int error, int lastError);

void setup() {

  //LLamado libreria

  Robot.SkyDragon();

  //Seleccion pines de salida 
  //13 -> buzzer

  pinMode(13, OUTPUT);
  //Asignación pines sensores sensores
  //Llamada libreria
  qtr.setTypeAnalog();

  //Asiganación pines para los sensores segun matriz
  qtr.setSensorPins((const uint8_t[]){ A5, A4, A3, A2, A1, A0 }, SensorCount);

  //Delay Millisegundos
  delay(200);

  // Calibracion sensores

  // Cantidad de toma de datos para la calibración
  for (uint16_t i = 0; i < 200; i++) {
    //Calibración establecida por libreria
    qtr.calibrate();
    if (digitalRead(4)) { linealSpeed = 20; }
    {}
  }

  // Asignación de baudios para el puerto de comunicación
  Serial1.begin(9600);

  //Valores minimos sensores

  for (uint8_t i = 0; i < SensorCount; i++) {
    //Impresión puerto serial de valores minimos obtenidos de los sensores segun calibración
    Serial1.print(qtr.calibrationOn.minimum[i]);
    Serial1.print(' ');
  }
  Serial1.println();

  //Valores maximos sensores

  for (uint8_t i = 0; i < SensorCount; i++) {
    //Impresión puerto serial de valores maximos obtenidos de los sensores segun calibración
    Serial1.print(qtr.calibrationOn.maximum[i]);
    Serial1.print(' ');
  }
  //Serial1 de espacio o tab
  Serial1.println();
  Serial1.println();

  //Indicador de terminado de calibración con buzzer
  digitalWrite(13, HIGH);
  delay(70);
  digitalWrite(13, LOW);

  //Delay Millisegundos
  delay(4000);

  Timer1.initialize(50000);  // inicializa el temporizador para 50 milisegundos
  Timer1.attachInterrupt(callback);  // adjunta la función callback

}

void callback() {
  //unsigned long startMillis = millis();  // obtiene el tiempo inicial
  

  // Voltaje Bateria
  //Asicnación de la variable y asiganacion pin del votaje de la bateria segun libreria -Sensor Bateria A11
  float batteryVoltage = Robot.Battery();

  //Asignación del valor posicion de linea  segun sensores leidos

  uint16_t position = qtr.readLineWhite(sensorValues);

  // Calculo valores PID
  // Sistema de control

  //Declaración error
  error = position - 2500;

  angularSpeed = pid(error, lastError, KP, KD, KI);

  lastError = error;

  //PID Velocidad motores

  //Declaración variables motores segun libreria y asisgancion de parametros
  int m1Speed = linealSpeed + angularSpeed;
  int m2Speed = linealSpeed - angularSpeed;

  //Activacion motores aplicando el Control PID
  //Segun libreria de la placa controladora
  //resolución de 8 bits (0-255)
  Robot.Speeds(m2Speed, m1Speed); 

  printData(batteryVoltage, KP, KD, error, angularSpeed);
  /*
  unsigned long endMillis = millis();  // obtiene el tiempo final
  unsigned long executionTime = endMillis - startMillis;  // calcula el tiempo de ejecución
  Serial1.print("Tiempo de ejecucion: ");
  Serial1.print(executionTime);
  Serial1.println(" miliseg");
  */

  if (Serial1.available() ) {  // Si hay datos disponibles en el puerto serial
    KD = Serial1.parseFloat(SKIP_ALL);
    Serial1.println("Cambio de KP ");
    Serial1.println(KD);
  }
}

void loop() {

  
}

int pid(int error, int lastError, float KP, float KI, float KD){

  //Declaración integral  
  int integral = (error + lastError);

  //Declaración derivativo
  int derivativo = (error - lastError);

  //Aplicación parametros
  if (integral > 1000) integral = 1000;

  if (integral < -1000) integral = -1000;

  //Formula PID

  float angularSpeed = (KP * error) + (KD * derivativo) + (KI * integral);

  return angularSpeed / 100;

}

void printData(float batteryVoltage, float KP, float KD, int error, int angularSpeed) {
  // imprime Los valores KP y KD, vBat, error, PID, m1Speed, m2Speed
  Serial1.print("Voltaje");
  Serial1.print('\t');
  //Impresión valor leido del batteryVoltage actual de la batería
  Serial1.print(batteryVoltage);
  Serial1.print('\t');

  Serial1.print("KP,KD");
  Serial1.print('\t');

  Serial1.print(KP);
  Serial1.print('\t');

  Serial1.print(KD);
  Serial1.print('\t');

    //Impresión de la palabra error
  Serial1.print("error");
  Serial1.print('\t');

  Serial1.print(error);
  Serial1.print('\t');

  Serial1.print("PID");
  Serial1.print('\t');
  Serial1.println(angularSpeed);
  Serial1.print('\t');
}



/*
SkyDragon es una tarjeta electronica diseñada para robotica general y basada en el arduino LEONARDO
Cuenta con controlador atmel del arduino Leonardo y su configuración

contiene las siguientes conexiones:
  *Entradas
    -Boton D4
    -Sensor Bateria A11
    - 9 pines digitales (A5,A4,A3,A2,A1,A0,D13,D10,D9), 6 pueden ser Analogos, 3 de PWM
  *Salidas
    -Motor MA(D6-D8)
    -Motor MB(D5-D7)
    -LED D11
  *Comunicacion
    -1 Serial Rx D0, Tx D1
    -1 I2C SDA D2, SCL D3
*/
