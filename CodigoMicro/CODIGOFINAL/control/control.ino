//***************************************************//
//***************************************************//
//*****   Posición y Velocidad de Motor DC      *****//
//*****                                         *****//
//***** by: Sergio Andres Castaño Giraldo       *****//
//***** https://controlautomaticoeducacion.com/ *****//
//*****                                         *****//
//***************************************************//
//***************************************************//

// this library includes the ATOMIC_BLOCK macro.
#include "SimplyAtomic.h"
#include "BTS7960.h"

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

//Objetos de envio y recepcion de datos

//Variables de recepción
FLOATUNION_t RefQ1;
FLOATUNION_t RefQ2;
FLOATUNION_t RefQ3;

//Variables de Envio
FLOATUNION_t q1M;
FLOATUNION_t q2M;
FLOATUNION_t q3M;
FLOATUNION_t Rq1;
FLOATUNION_t Rq2;
FLOATUNION_t Rq3;


const uint8_t EN = 11;
const uint8_t L_PWM = 13;
const uint8_t R_PWM = 4;

//const uint8_t EN2 = 10;
//const uint8_t L_PWM2 = 9;
//const uint8_t R_PWM2 = 8;

BTS7960 motorController(EN, L_PWM, R_PWM);
//BTS7960 motorController2(EN2, L_PWM2, R_PWM2);

#define ENCODER_A 2  // Amarillo
#define ENCODER_B 3  // Verde

#define ENCODER_A2 18  // Amarillo
#define ENCODER_B2 19  // Verde

boolean sentido1 = true;
boolean sentido2 = true;

//Constantes
float posicion;
float posicion2;

float rpm;
float rpm2;

int value, dir = true;
int value2, dir2 = true;

//Compensación gravitacional

float m1 = 0.163;  //Kg
float l1 = 0.187;  //m
float lc1 = 0.1178;
float I1 = 0.0012;  //Nm/s2

float m2 = 0.1038;  //kg
float l2 = 0.181;   //m
float lc2 = 0.0994;
float I2 = 5.5942e-4;

float g = 9.81;
// Pines de Control L298N
//#define RPWM 13
//#define LPWM 12
//#define PWM 11

#define RPWM2 8
#define LPWM2 9
#define EN2 10


//Variable global de posición compartida con la interrupción
volatile int theta = 0;
volatile int theta2 = 0;

//Variable global de pulsos compartida con la interrupción
volatile int pulsos = 0;
unsigned long timeold;

volatile int pulsos2 = 0;
unsigned long timeold2;

long long previousMillis;

float resolution = 285.22;
//float resolution = 374.22;
int duracion = -1000;

//Variable Global Velocidad
int vel = 0;
int vel2 = 0;

//Variable Global Posicion
int ang = 0;
int ang2 = 0;

//Variable Global MODO
bool modo = false;

//Constantes de Controlador
float kp1 = 4.63*3;
float CmdP = 0;
unsigned int pwmDuty = 0;
int pwmMax = 255;
int pwmMin = 85;
int pwmMin2 = 85;
float E1 = 0;
float Ref1 = 120;

//Constantes de Controlador
float kp2 = 12;//2.22;//9.65;//0.84;
float CmdP2 = 0;
unsigned int pwmDuty2 = 0;
float E2 = 0;
float Ref2 = -100;

long int cmillis;
long int cmillis2;


float Ts = 4;
float TsC = 1;

void setup() {
  // set timer 1 divisor to  1024 for PWM frequency of 30.64 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;
  Serial.begin(115200);
  //Serial.begin(9600);

  RefQ1.number = 180;
  RefQ2.number = -165;

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Leonardo only
  }
  //Encoders como entradas
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  //Configura Motor
  //digitalWrite (LPWM,LOW);
  //digitalWrite (RPWM,LOW);
  //pinMode(LPWM,OUTPUT);
  //pinMode(RPWM,OUTPUT);
  //pinMode(PWM,OUTPUT);

  digitalWrite(LPWM2, LOW);
  digitalWrite(RPWM2, LOW);
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(EN2, OUTPUT);

  //Configurar Interrupción
  timeold = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), leerEncoder, RISING);

  timeold2 = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), leerEncoder2, RISING);
  delay(3000);
}

void loop() {
  motorController.Enable();

  if (millis() - cmillis2 > 5) {
    //Recepción
    Recepcion();
    cmillis2 = millis();
  }

  //Controlador
  controlP();

  if (millis() - cmillis > 5) {

    //Envio
    Envio();
    cmillis = millis();
  }
}

//Función para dirección y velocidad del Motor
void setMotor(int motor, int vel, bool dir) {
  if (motor == 1) {
    if (dir) {
      motorController.TurnRight(vel);
    } else {
      motorController.TurnLeft(vel);
    }
  } else if (motor == 2) {
    digitalWrite(EN2, HIGH);
    if (dir) {
      analogWrite(LPWM2, 0);
      analogWrite(RPWM2, 0);
      analogWrite(RPWM2, vel);
    } else {
      analogWrite(LPWM2, 0);
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, vel);
    }
  } else {
    Serial.print("mm a");
  }
}

//Función anti-rebote
bool debounce(byte input) {
  bool state = false;
  if (!digitalRead(input)) {
    delay(200);
    while (!digitalRead(input))
      ;
    delay(200);
    state = true;
  }
  return state;
}

//Función para la lectura del encoder
void leerEncoder() {
  //Lectura de Velocidad
  if (modo)
    pulsos++;  //Incrementa una revolución

  //Lectura de Posición
  else {
    int b = digitalRead(ENCODER_B);
    if (b > 0) {
      //Incremento variable global
      theta++;
    } else {
      //Decremento variable global
      theta--;
    }
  }
}

void leerEncoder2() {
  //Lectura de Velocidad
  if (modo)
    pulsos2++;  //Incrementa una revolución

  //Lectura de Posición
  else {
    int b = digitalRead(ENCODER_B2);
    if (b > 0) {
      //Incremento variable global
      theta2++;
    } else {
      //Decremento variable global
      theta2--;
    }
  }
}

void controlP() {
  unsigned long currentMillis = millis();
  //Ref2= 0;
      if (currentMillis >= 5000) {

      ATOMIC() {
        posicion = (float(-theta * 360.0 / resolution)) + 180;
        posicion2 = (float(theta2 * 360.0 / resolution)) - posicion + 15;
      }

      E1 = (float(RefQ1.number) - posicion);
      E2 = (float(RefQ2.number) - posicion2);

      if (abs(E1) <= 1) {
        E1 = 0;
      }

      if (abs(E2) <= 0.1) {
        E2 = 0;
      }

      CmdP = kp1 * (E1);
      CmdP2 = kp2 * (E2);
    }
  if (currentMillis - previousMillis >= TsC) {
    previousMillis = currentMillis;

    float Cmd = CmdP + 260.0286 * ((g * m2 * l1 * cos((posicion) * (PI / 180)) + g * m2 * lc2 * cos((posicion2 + posicion) * (PI / 180)) + g * lc1 * m1 * cos((posicion) * (PI / 180))));  //+ g*m2*l1*cos(posicion)+g*m2*lc2*cos(posicion+posicion2)+g*lc1*m1*cos(posicion);
    float Cmd2 = CmdP2 + (g * lc2 * m2 * cos((posicion + posicion2) * (PI / 180))) * 260.0286;

    if (Cmd >= 0) {
      sentido1 = true;
    } else {
      sentido1 = false;
    }

    if (Cmd2 >= 0) {
      sentido2 = false;
    } else {
      sentido2 = true;
    }

    Cmd = abs(Cmd);
    Cmd2 = abs(Cmd2);

    float CmdLim = min(max(Cmd, 0), pwmMax);    // Saturated Control Output
    float CmdLim2 = min(max(Cmd2, 0), pwmMax);  // Saturated Control Output

    pwmDuty = int(CmdLim);    //int((CmdLim/1)*pwmMax);//int(CmdLim);//int((CmdLim/1)*pwmMax);
    pwmDuty2 = int(CmdLim2);  //int((CmdLim2/1)*pwmMax);

    if (currentMillis >= 5000) {
      setMotor(1, pwmDuty, sentido1);
      // setMotor(1,150,false);
      setMotor(2, pwmDuty2, sentido2);
    }




    // Serial.print(E1);
    // Serial.print(",");
    // Serial.println(E2);
    //delay(1000);
  }
}


float getFloat() {
  int cont = 0;
  FLOATUNION_t f;
  while (cont < 4) {  // ller 4 bytes (32bits por numero)
    f.bytes[cont] = Serial.read();
    cont = cont + 1;
  }
  return f.number;
}


void Envio() {

  q1M.number = float(posicion);
  q2M.number = float(posicion2);
  q3M.number = 0;
  Rq1.number = float(RefQ1.number);
  Rq2.number = float(RefQ2.number);
  Rq3.number = float(RefQ3.number);
  // while(Serial.availableForWrite()<1){

  // }

  // Print header: Important to avoid sync errors!
  if (Serial.availableForWrite() > 0) {
    Serial.write('A');
  }
  // Print float data
  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(q1M.bytes[i]);
    }
  }
  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(q2M.bytes[i]);
    }
  }
  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(q3M.bytes[i]);
    }
  }
  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(Rq1.bytes[i]);
    }
  }
  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(Rq2.bytes[i]);
    }
  }
  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(Rq3.bytes[i]);
    }
  }
  // Print terminator
  if (Serial.availableForWrite() > 0) {
    Serial.print('\n');
  }
  //long int cmillis = millis();
  //while(millis()-cmillis < 50){}
}

void Recepcion() {
  if (Serial.available() > 1) {
    Serial.find('R');
    //if(Serial.read()=='R'){
    //RefQ1.number = Serial.parseFloat();
    float aux = getFloat();
    if(!isnan(aux) && abs(aux-RefQ1.number)<20){
      RefQ1.number = aux;
    }
    

    aux = getFloat();
    if(!isnan(aux)&& abs(aux-RefQ2.number)<20){
      RefQ2.number = aux;
    }

    aux = getFloat();
    if(!isnan(aux)&& abs(aux-RefQ3.number)<20){
      RefQ3.number = aux;
    };
    

    Serial.find('\n');
  }
}