#include <stdint.h>
#include <stdbool.h>

#define PART_TM4C123GH6PM
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

// Variables globales
const int escala = 10;
const int pot = A1;
const int motor = PB_3;
const int boton2 = PF_0;  //PUSH2 
const int boton1 = PF_4; //PUSH1
const int umbral = 150; // umbral de valor analogico del pote para iniciar el motor
uint32_t seg=0; //segundos para el Timer de capa 8
uint32_t status=0; // estado de los pulsadores
uint16_t minutos_t=0; // minutos del temporizador
uint8_t minutos_s, horas_s; // minutos y horas para mostrar por LCD
boolean controlmotor=0; // bandera de si el motor esta prendido o no
uint16_t pwm; // duty del motor controlador por el Potenciometro
boolean temporizadorOFF = 1, cuentareset = 0;


void initTimer()
{  // se habilita despues de 5 ciclos de reloj, calcular para no escribirlo antes)
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //habilito el timer0
  ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);   // 32 bits Timer
  TimerIntRegister(TIMER0_BASE, TIMER_A, TimerIsr);    // Registro la ISR con el nombre TimerIsr     
  ROM_TimerEnable(TIMER0_BASE, TIMER_A); 
  ROM_IntEnable(INT_TIMER0A); 
  ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  
}

void TimerIsr(void)
{
  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Levanta la interrupcion
  seg++; // si estamos a 30Hz - cuenta 1 seg
  //digitalWrite(RED_LED, digitalRead(RED_LED) ^ 1); // Toggle led rojo - Control
}

void initbotones()
{
  GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
  GPIOIntRegister(GPIO_PORTF_BASE,BotonesIsr);
  GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
  GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
}

void BotonesIsr(){
  delay(50);
  status = GPIOIntStatus(GPIO_PORTF_BASE,true);
  GPIOIntClear(GPIO_PORTF_BASE,status); // Levanta la interrupcion 
  Serial.print("status=");
  Serial.println(status); //DEBUG
 // control = 0; //solo si presiono programar
 
if( (status & GPIO_INT_PIN_4) == GPIO_INT_PIN_4){
  delay(50);
  minutos_t++;
//    temporizadorOFF = false; //solo si presiono programar
    if (minutos_t > 1440)minutos_t=0;
    Serial.print("Minutos: ");  
    Serial.println(minutos_t);
    } 
  
  if( (status & GPIO_INT_PIN_0) == GPIO_INT_PIN_0){
   // control = 1;
    delay(50);
    minutos_t--;
    if (minutos_t > 1440 && minutos_t <= 65535)minutos_t=1440;
    //if (
    Serial.print("Minutos: "); 
    Serial.println(minutos_t);
    } 


}


void setup()
{
  Serial.begin(9600); //Comunicacion
  pinMode(RED_LED,OUTPUT); // Para la ISR del timer
  initTimer(); //inicializo el timer
  pinMode(boton1, INPUT_PULLUP);
  pinMode(boton2, INPUT_PULLUP);
  pinMode(GREEN_LED,OUTPUT);// para DEBUG
  pinMode(pot, INPUT);
  pinMode(motor,OUTPUT);
  initbotones();
 
}

void loop() 
{
  unsigned long ulPeriod;
  unsigned int Hz = 30;   // frequency in Hz  
  ulPeriod = (SysCtlClockGet() / Hz)/ 2;
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A,ulPeriod -1);
  
  while (1)
  {
      pwm=pote();
    //  motor_ON(pwm);

      if( !digitalRead(boton1) && !digitalRead(boton2)){
        temporizadorOFF = !temporizadorOFF;
        Serial.print("tempOFF=");
        Serial.println(temporizadorOFF);
        
        delay (2000);
  //      control = 0;
        }


      temporizador(); // prendo motores durante X minutos
//      control = 1;
      
  
 //   min_a_hs(minutos_t);
    
    if (!digitalRead(boton1)){ //si apreto el boton, prendo LED)
      digitalWrite(GREEN_LED,HIGH);
    }
    else{    //si no apreto, hago lo de siempre
      digitalWrite(GREEN_LED,LOW);
      Serial.println(seg/60); 
     // Serial.println(pwm); 
      delay(1000);
      }
  }
}

// funciones para usuario

uint32_t pote(){ // el pote se conecta en PE_2 o A1
  uint32_t duty;
  if ( analogRead(pot) > umbral ) //umbral se seteara globalmente
    duty = analogRead(pot) / escala;
  return duty;
}

void motor_ON(uint32_t duty){
  
 if (duty >=245) duty=245; // Rango max 96% PWM
 if (duty <= 60) duty=60;  // Rango min 19,6% PWM
 
 if (temporizadorOFF){
   analogWrite(motor,duty);
   Serial.println("Motor ON en PB_3"); //DEBUG
   Serial.print("Duty manual:");
   Serial.println(duty); //DEBUG
 }
 while(duty < 250 && !temporizadorOFF){ // prendido automatico lento
    analogWrite(motor,duty);
    Serial.print("Duty auto on:");
    Serial.println(duty);
   // Serial.println("Motor ON en PB_3"); //DEBUG 
    duty++;
    delay(250);  
    }
}

void motor_OFF(uint32_t duty){
  
  while(duty > 0){ 
   analogWrite(motor,duty);
   Serial.print("Duty auto off:"); //DEBUG
   Serial.println(duty); //DEBUG
   Serial.println("Motor OFF en PB_3"); //DEBUG 
   duty--;
   delay(250);  // variar este valor para apagado mas rapido o lento
  }
 }


// Funciones relacionadas al Tiempo 
void temporizador (){
    if (!temporizadorOFF && !cuentareset) //si el temp se encendio chequear el tiempo
      if (seg >=0){
        seg=0;
        cuentareset=1;
      }
      if(!temporizadorOFF && cuentareset){
      if ( seg/60 == minutos_t *60){
        controlmotor = 1;  
        temporizadorOFF = true;
        if (temporizadorOFF)
          Serial.println("TempOFF: True;Apagado");
        } 
    }
    if (!controlmotor && !temporizadorOFF){
     Serial.println("Prendemos motor"); //debug
     //motor_ON(pwm);
     controlmotor=1;
    }
      
    if (controlmotor && temporizadorOFF){
         Serial.println("Apagamos motor"); //debug
 //      motor_OFF(pwm);
         controlmotor=0;
         cuentareset=1;
    }
  }

  
void min_a_hs(uint16_t min){
	horas_s=min/60;
        minutos_s=min-(horas_s*60);
	Serial.print("Horas convertidas:");
	Serial.println(horas_s);
        Serial.print("Minutos convertidos:");
        Serial.println(minutos_s);
}
