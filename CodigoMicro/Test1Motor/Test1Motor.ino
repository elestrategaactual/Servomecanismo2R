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
#include <util/atomic.h>

#define ENCODER_A       2 // Amarillo
#define ENCODER_B       3 // Verde

#define ENCODER_A2       18 // Amarillo
#define ENCODER_B2       19 // Verde


// Pines de Control L298N
#define RPWM 13
#define LPWM 12
#define PWM 11

#define RPWM2 8
#define LPWM2 9
#define PWM2 10


//Variable global de posición compartida con la interrupción
volatile int theta = 0;
volatile int theta2 = 0;

//Variable global de pulsos compartida con la interrupción
volatile int pulsos = 0;
unsigned long timeold;

volatile int pulsos2 = 0;
unsigned long timeold2;


float resolution = 285.22;

int duracion = 0;

//Variable Global Velocidad
int vel = 0;
int vel2 = 0;

//Variable Global Posicion
int ang = 0;
int ang2 = 0;

//Variable Global MODO
bool modo = false;

void setup(){
  // set timer 1 divisor to  1024 for PWM frequency of 30.64 Hz
  TCCR1B = TCCR1B & B11111000 | B00000101;
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  //Encoders como entradas
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  //Configura Motor
  digitalWrite (LPWM,LOW);
  digitalWrite (RPWM,LOW);
  pinMode(LPWM,OUTPUT);
  pinMode(RPWM,OUTPUT);
  pinMode(PWM,OUTPUT);
  
  digitalWrite (LPWM2,LOW);
  digitalWrite (RPWM2,LOW);
  pinMode(LPWM2,OUTPUT);
  pinMode(RPWM2,OUTPUT);
  pinMode(PWM2,OUTPUT);
  
  //Configurar Interrupción
  timeold = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A),leerEncoder,RISING);

  timeold2 = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2),leerEncoder2,RISING);
  delay(5000);
}

void loop(){
  float posicion;
  float posicion2;
  
  float rpm;
  float rpm2;
  
  int value, dir=true;
  int value2, dir2=true;
  
  if(modo){

    //Espera un segundo para el calculo de las RPM
    if (millis() - timeold >= 10)
   {
      //Modifica las variables de la interrupción forma atómica
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        rpm = float(pulsos * 60.0 / resolution); //RPM
        rpm = float((60.0 * 1000.0 / resolution ) / (millis() - timeold) * pulsos);
        //timeold = millis();
        pulsos = 0;
      }
      Serial.print("RPM: ");
      Serial.println(rpm);
      //Serial.print("Voy a por ti ostia lucas");
      //Serial.println(vel);
      
    }int PWMa = 0;
    if(duracion<12){
      PWMa = 250;
    }else if(duracion<50){
      PWMa = 120;
    }else if(duracion<70){
      PWMa = 100;
    }else if(duracion<130){
      PWMa = 75;
    }else if(duracion<140){
      PWMa = 50;
    }else if(duracion<150){
      PWMa = 25;
    }
    else{
      PWMa = 0;
    }
    if(duracion<1000){
      duracion++;
      delay(10);
      setMotor(1,PWMa,false);
      
    }
  }
  else{    
    //Modifica las variables de la interrupción forma atómica
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      posicion = (float(theta * 360.0 /resolution));
      posicion2 = (float(theta2 * 360.0 /resolution));
    }
    int PWMa = 0;
    if(duracion<25){
      PWMa = 250;
    }else if(duracion<100){
      PWMa = 120;
    }else if(duracion<110){
      PWMa = 100;
    }else if(duracion<130){
      PWMa = 75;
    }else if(duracion<140){
      PWMa = 50;
    }else if(duracion<800){
      PWMa = 100;
    }
    else{
      PWMa = 0;
    }
    if(duracion<1000){
      duracion++;
      delay(5);
      setMotor(1,PWMa,false);
      
    }
    Serial.print(posicion);
    Serial.print(" , ");
    Serial.println(PWMa);
    
    
  }
}

//Función para dirección y velocidad del Motor
void setMotor(int motor, int vel, bool dir){
  if(motor==1){
    if(dir){
      digitalWrite (LPWM,LOW);
      digitalWrite (RPWM,LOW);
      digitalWrite (RPWM,HIGH);
      analogWrite(PWM,vel);
    }else{
      digitalWrite (LPWM,LOW);
      digitalWrite (RPWM,LOW);
      digitalWrite (LPWM,HIGH);
      analogWrite(PWM,vel);
    }    
  }
  else if(motor==2){
    if(dir){
      digitalWrite (LPWM2,LOW);
      digitalWrite (RPWM2,LOW);
      digitalWrite (RPWM2,HIGH);
      analogWrite(PWM2,vel2);
    }else{
      digitalWrite (LPWM2,LOW);
      digitalWrite (RPWM2,LOW);
      digitalWrite (LPWM2,HIGH);
      analogWrite(PWM2,vel2);
    }
  }else{
    Serial.print("mm a");
  }
  
}

//Función anti-rebote
bool debounce(byte input){
  bool state = false;
  if(! digitalRead(input)){
    delay(200);
    while(! digitalRead(input));
    delay(200);
    state = true;
  }      
  return state;   
}

//Función para la lectura del encoder
void leerEncoder(){
  //Lectura de Velocidad
  if(modo)
    pulsos++; //Incrementa una revolución
    
  //Lectura de Posición  
  else{
    int b = digitalRead(ENCODER_B);
    if(b > 0){
      //Incremento variable global
      theta++;
    }
    else{
      //Decremento variable global
      theta--;
    }
  }
}

void leerEncoder2(){
  //Lectura de Velocidad
  if(modo)
    pulsos2++; //Incrementa una revolución
    
  //Lectura de Posición  
  else{
    int b = digitalRead(ENCODER_B2);
    if(b > 0){
      //Incremento variable global
      theta2++;
    }
    else{
      //Decremento variable global
      theta2--;
    }
  }
}
