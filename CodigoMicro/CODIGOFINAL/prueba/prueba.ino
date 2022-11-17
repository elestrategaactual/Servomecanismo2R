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

typedef union{
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
float kp1 = 4.63;
double CmdP = 0;
unsigned int pwmDuty = 0;
int pwmMax = 255;
int pwmMin = 85;
int pwmMin2 = 85;
double E1 = 0;
double Ref1 = 120;

//Constantes de Controlador
float kp2 = 0.84;
double CmdP2 = 0;
unsigned int pwmDuty2 = 0;
double E2 = 0;
double Ref2 = -100;

float Ts = 4;
float TsC = 1;

void setup() {
  // set timer 1 divisor to  1024 for PWM frequency of 30.64 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;
 Serial.begin(115200);
 //Serial.begin(9600);
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

  prueba();

  //Controlador
  //controlP();
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
      //motorController2.TurnRight(vel);
      analogWrite(LPWM2, 0);
      analogWrite(RPWM2, 0);
      analogWrite(RPWM2, vel);
    } else {
      //motorController2.TurnLeft(vel);
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
  if (currentMillis - previousMillis >= TsC) {
    previousMillis = currentMillis;

    double Cmd = CmdP + 260.0286 * ((g * m2 * l1 * cos((posicion) * (PI / 180)) + g * m2 * lc2 * cos((posicion2 + posicion) * (PI / 180)) + g * lc1 * m1 * cos((posicion) * (PI / 180))));  //+ g*m2*l1*cos(posicion)+g*m2*lc2*cos(posicion+posicion2)+g*lc1*m1*cos(posicion);
    double Cmd2 = CmdP2 + (g * lc2 * m2 * cos((posicion + posicion2) * (PI / 180))) * 260.0286;

    if (Cmd >= 0) {
      sentido1 = true;
    } else {
      sentido1 = false;
    }

    if (Cmd2 >= 0) {
      sentido2 = true;
    } else {
      sentido2 = false;
    }

    Cmd = abs(Cmd);
    Cmd2 = abs(Cmd2);

    double CmdLim = min(max(Cmd, 0), pwmMax);    // Saturated Control Output
    double CmdLim2 = min(max(Cmd2, 0), pwmMax);  // Saturated Control Output

    pwmDuty = int(CmdLim);    //int((CmdLim/1)*pwmMax);//int(CmdLim);//int((CmdLim/1)*pwmMax);
    pwmDuty2 = int(CmdLim2);  //int((CmdLim2/1)*pwmMax);

    if (currentMillis >= 5000) {
      setMotor(1, pwmDuty, sentido1);
      // setMotor(1,150,false);
      setMotor(2, pwmDuty2, sentido2);
    }



    if (currentMillis >= 5000) {

      ATOMIC() {
        posicion = (float(-theta * 360.0 / resolution)) + 180;
        posicion2 = (float(theta2 * 360.0 / resolution)) - posicion + 15;
      }

      E1 = (Ref1 - posicion);
      E2 = (Ref2 - posicion2);

      if (abs(E1) <= 1) {
        E1 = 0;
      }

      if (abs(E2) <= 1) {
        E2 = 0;
      }

      CmdP = kp1 * (E1);
      CmdP2 = kp2 * (E2);
    }

    //Motor 1
    Serial.print(currentMillis / 1000.0, DEC);
    Serial.print(",");
    Serial.print(Ref1, DEC);
    Serial.print(",");
    Serial.print("angle: ");
    Serial.print(posicion, DEC);
    Serial.print(",");
    Serial.print("E: ");
    Serial.print(E1, DEC);
    Serial.print(",");
    Serial.print("s: ");
    Serial.print(sentido1, DEC);
    Serial.print(",");
    Serial.print("pwmC: ");
    Serial.print(CmdP, DEC);
    Serial.print(",");
    Serial.print("PWM: ");
    Serial.println(pwmDuty, DEC);

    //delay(1000);

    //Motor 2
    Serial.print("Motor 2");
    Serial.print(",");
    Serial.print(Ref2, DEC);
    Serial.print(",");
    Serial.print("angle2: ");
    Serial.print(posicion2, DEC);
    Serial.print(",");
    Serial.print("E: ");
    Serial.print(E2, DEC);
    Serial.print(",");
    Serial.print("s: ");
    Serial.print(sentido2, DEC);
    Serial.print(",");
    Serial.print("pwmC: ");
    Serial.print(CmdP2, DEC);
    Serial.print(",");
    Serial.print("PWM: ");
    Serial.println(pwmDuty2, DEC);

    delay(1000);
  }
}


void prueba() {
  bool modo = false;
  if (modo) {

    //Espera un segundo para el calculo de las RPM
    if (millis() - timeold >= 10) {
      //Modifica las variables de la interrupción forma atómica
      ATOMIC() {
        rpm = float(pulsos * 60.0 / resolution);  //RPM
        rpm = float((60.0 * 1000.0 / resolution) / (millis() - timeold) * pulsos);
        //timeold = millis();
        pulsos = 0;
      }
      Serial.print("RPM: ");
      Serial.println(rpm);
      //Serial.print("Voy a por ti ostia lucas");
      //Serial.println(vel);
    }
    int PWMa = 0;
    if (duracion < 12 && duracion > 0) {
      PWMa = 250;
    } else if (duracion < 50) {
      PWMa = 120;
    } else if (duracion < 70) {
      PWMa = 100;
    } else if (duracion < 130) {
      PWMa = 75;
    } else if (duracion < 140) {
      PWMa = 50;
    } else if (duracion < 150) {
      PWMa = 25;
    } else {
      PWMa = 0;
    }
    
    if (duracion < 1000) {
      duracion++;
      delay(10);
      setMotor(1, PWMa, true);
    }
  } else {
    //Modifica las variables de la interrupción forma atómica
    ATOMIC() {
      posicion = (float(theta * 360.0 / resolution));
      posicion2 = (float(theta2 * 360.0 / resolution));
    }
    //    int PWMa = 0;
    //    if(duracion>0){
    //      if(duracion<60){
    //        PWMa = 240;
    //      }else if(duracion<500){
    //        PWMa = 0;
    //      }else if(duracion<595){
    //        PWMa = 210;
    //      }else if(duracion<1000){
    //        PWMa = 0;
    //      }else if(duracion<1060){
    //        PWMa = 200;
    //      }else if(duracion<1500){
    //        PWMa = 0;
    //      }else if(duracion<1555){
    //        PWMa = 190;
    //      }else if(duracion<2000){
    //        PWMa = 0;
    //      }
    //      else{
    //        PWMa = 0;
    //      }
    //    }
    //    if(duracion<2000){
    //      duracion++;
    //      delay(2);
    //      setMotor(2,PWMa,true);
    //
    //    }
    //int PWMa = 0;
    //if (duracion > 0) {
    //  if (duracion < 50) {
    //    PWMa = 200;
    //  } else if (duracion < 500) {
    //    PWMa = 0;
    //  } else if (duracion < 565) {
    //    PWMa = 220;
    //  } else if (duracion < 1000) {
    //    PWMa = 0;
    //  } else if (duracion < 1110) {
    //    PWMa = 190;
    //  } else if (duracion < 1500) {
    //    PWMa = 0;
    //  } else if (duracion < 1550) {
    //    PWMa = 210;
    //  } else if (duracion < 2000) {
    //    PWMa = 0;
    //  } else {
    //    PWMa = 0;
    //  }
    //}

    int PWMa = 0;
    if(millis()<5000){
      PWMa = 0;
    } else if(millis()<5180){
      PWMa = 200;
    } else {
      PWMa = 0;
    }
    
    setMotor(2, PWMa, true);
    Serial.print(float(micros()));
    Serial.print(",");
    Serial.print(float(posicion2));
    Serial.print(",");
    Serial.println(float(PWMa));
  }
}