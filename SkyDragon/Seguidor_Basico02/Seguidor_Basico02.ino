//librerias
#include <sky.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

SoftwareSerial Serial_1(9, 1);

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

int lastError = 0;
int M1 = 0;
int M2 = 0;
int V = 0;
int W;


void setup() {

  //LLamado libreria

  Robot.SkyDragon();

  //Seleccion pines de salida
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);
  //Asignación pines sensores sensores
  //Llamada libreria
  qtr.setTypeAnalog();

  //Asiganación pines para los sensores segun matriz
  qtr.setSensorPins((const uint8_t[]){ A5, A4, A3, A2, A1, A0 }, SensorCount);

  //Asignación pin emisor
  //qtr.setEmitterPin(9);

  //Delay Millisegundos
  delay(200);

  //Asignación y encendido led indicador de inicio de calibración de sensores
  digitalWrite(11, HIGH);
  // Calibracion sensores

  // Cantidad de toma de datos para la calibración
  for (uint16_t i = 0; i < 200; i++) {
    //Calibración establecida por libreria
    qtr.calibrate();
    if (digitalRead(4)) { V = 20; }
    {}
  }
  //Asignación y apagado led indicador de fin de calibración de sensores
  digitalWrite(11, LOW);

  // Asignación de baudios para el puerto de comunicación
  Serial_1.begin(9600);

  //Valores minimos sensores

  for (uint8_t i = 0; i < SensorCount; i++) {
    //Impresión puerto serial de valores minimos obtenidos de los sensores segun calibración
    Serial_1.print(qtr.calibrationOn.minimum[i]);
    Serial_1.print(' ');
  }
  Serial_1.println();

  //Valores maximos sensores

  for (uint8_t i = 0; i < SensorCount; i++) {
    //Impresión puerto serial de valores maximos obtenidos de los sensores segun calibración
    Serial_1.print(qtr.calibrationOn.maximum[i]);
    Serial_1.print(' ');
  }
  //Serial_1 de espacio o tab
  Serial_1.println();
  Serial_1.println();

  //tone(13, 200,20);
  digitalWrite(13, HIGH);
  delay(70);
  digitalWrite(13, LOW);

  //Delay Millisegundos
  delay(4000);

  /*
  
  //Espera a que se presione el boton 14
  while(digitalRead(14)==0){

  //Asignación y encendido led 
  digitalWrite(0, HIGH);  
  delay(1000);   

  //Asignación y encendido led            
  digitalWrite(0, LOW);  
  
  //Imoresión del estado de ON o OFF
  Serial_1.print("ON");
  Serial_1.println('\t');
  }

  //Retardo de 2 segundos
  delay(2000);
  
  */
}

void loop() {

  if (Serial_1.available() > 0) {  // Si hay datos disponibles en el puerto serial
    KD = Serial_1.parseFloat(SKIP_ALL);
    Serial_1.println("Cambio de KP ");
    Serial_1.println(KD);
    delay(1000);
  }

  // Voltaje Bateria
  //Asicnación de la variable y asiganacion pin del votaje de la bateria segun libreria -Sensor Bateria A11
  float voltaje = Robot.Battery();

  //Impresión de la palabra voltaje
  Serial_1.print("Voltaje");
  Serial_1.print('\t');
  //Impresión valor leido del voltaje actual de la batería
  Serial_1.print(voltaje);
  Serial_1.print('\t');

  Serial_1.print(KD);
  Serial_1.print('\t');

  //Asignación del valor posicion de linea  segun sensores leidos

  uint16_t position = qtr.readLineWhite(sensorValues);
  /*
  for (uint8_t i = 0; i < SensorCount; i++) {

    //Impresion de la palabra sensores
    Serial_1.print("Sensores");
    Serial_1.print('\t');

    //Impresion de los valores de los sensores en tiempo real utilizando matriz y libreria
    Serial_1.print(sensorValues[i]);
    Serial_1.print('\t');
  }
  */
  // Calculo valores PID
  // Sistema de control

  //Declaración error
  // error = linproporcional
  int error = position - 2500;

  //Declaración integral
  int integral = (error + lastError);

  //Declaración derivativo
  int derivativo = (error - lastError);

  //Aplicación parametros
  if (integral > 1000) integral = 1000;

  if (integral < -1000) integral = -1000;

  //Impresión de la palabra Posición
  //El valor esta establecido en formula asignada en la libreria
  //lINK Control PID
  Serial_1.print("Position");
  Serial_1.print('\t');
  //Impresión del valor de la posicion respecto a la linea
  Serial_1.print(position);
  Serial_1.print('\t');

  //Formula PID

  W = (KP * error) + (KD * (derivativo)) + (integral * KI);
  lastError = error;

  W = W / 100;

  //Impresión de las palabras y valores despues de aplicar las formulas y parametros establecidos en el control PID

  //Impresión de la palabra Proporcional
  Serial_1.print("error");
  Serial_1.print('\t');

  //Impresión del valor proporcional, asignado al error en el PID
  Serial_1.print(error);
  Serial_1.print('\t');

  Serial_1.print("PID");
  Serial_1.print('\t');
  Serial_1.println(W);
  Serial_1.print('\t');
  /*
  //Impresión de la palabra derivativo
  Serial_1.print("Derivativo");
  Serial_1.print('\t');

  //Impresión del valor  derivativo
  Serial_1.print(derivativo);
  Serial_1.print('\t');

  //Impresión de la palabra integral
  Serial_1.print("Integral");
  Serial_1.print('\t');

  //Impresión del valor integral
  Serial_1.println(integral);
  Serial_1.print('\t');
*/
  //PID Velocidad motores

  //Declaración variables motores segun libreria y asisgancion de parametros
  int m1Speed = V + W;
  int m2Speed = V - W;

  //Parametro de velocidad menor a 0 == velocidad en motor sera 0 - Evita cortos o corrientes paracitas
  //Motor A
  /*
  if (m1Speed < 0)
    m1Speed = 0;

  //Motor B
  if (m2Speed < 0)
    m2Speed = 0;
  */

  //Activacion motores aplicando el Control PID
  //Segun libreria de la placa controladora

  Robot.Speeds(m2Speed, m1Speed);  //50 es valor de compensación diferencia de potencia de motor
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
