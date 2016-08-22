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
uint8_t status=0; // estado de los pulsadores
uint16_t minutos_t=0; // minutos del temporizador
uint8_t minutos_s=0, horas_s=0, dutyauto=0; // minutos y horas para mostrar por LCD
boolean controlmotor=0; // bandera de si el motor esta prendido o no
uint16_t pwm; // duty del motor controlador por el Potenciometro
boolean temporizadorOFF = 1, cuentareset = 0, dutymanual = 1;


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
//  Serial.print("status=");
//  Serial.println(status); //DEBUG
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
    delay(50);
    minutos_t--;
    if (minutos_t > 1440 && minutos_t <= 65535)minutos_t=1440;
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
    motor_ON(pwm);

      if( !digitalRead(boton1) && !digitalRead(boton2)){
        temporizadorOFF = !temporizadorOFF;
        Serial.print("tempOFF=");
        Serial.println(temporizadorOFF);
        delay (2000);
        }


      temporizador(); // prendo motores durante X minutos
      
      Serial.print("Segundos:");
      Serial.println(seg/60);
      delay(1000); 
  
 //   min_a_hs(minutos_t);
   
  }
}

// funciones para usuario

uint32_t pote(){ // el pote se conecta en PE_2 o A1
  uint32_t duty;
  if ( analogRead(pot) > umbral ) //umbral se seteara globalmente
    duty = analogRead(pot) / escala;
  return duty;
}


void motor_ON(uint32_t dutym){
  
  uint8_t dutyauto=50;
  boolean escape=0;
  
 // if(dutym > 0 || dutyauto > 0)
 // controlmotor=1;
 // else
 // controlmotor=0;
  
 if (dutym >=250) dutym=250; // Rango max 96% PWM
 if (dutym <= 60) dutym=60;  // Rango min 19,6% PWM
 
 if (temporizadorOFF){
   analogWrite(motor,dutym);
   Serial.println("Motor ON en PB_3"); //DEBUG
   Serial.print("Duty manual:");
   Serial.println(dutym); //DEBUG
 }
 while(dutyauto < 250 && !temporizadorOFF){ // prendido automatico lento
    if(!escape){
    dutyauto=50;
    escape=1;
    }
    analogWrite(motor,dutyauto);
    Serial.print("Duty auto on:");
    Serial.println(dutyauto);
   // Serial.println("Motor ON en PB_3"); //DEBUG 
    dutyauto++;
    delay(200);  
    }
    if (dutyauto<=250)escape=0;
}

void motor_OFF(){
  if(dutyauto == 0) dutyauto=250;
  
  do{ 
   analogWrite(motor,dutyauto);
   Serial.print("Duty auto off:"); //DEBUG
   Serial.println(dutyauto); //DEBUG
 //  Serial.println("Motor OFF en PB_3"); //DEBUG 
   dutyauto--;
   delay(210);  // variar este valor para apagado mas rapido o lento
  }while(dutyauto > 0);
  
  
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
     motor_ON(pwm); // en un futuro desdoblar las funciones en on auto y manual
     controlmotor=1;
    }
      
    if (controlmotor && temporizadorOFF){
         Serial.println("Apagamos motor"); //debug
         delay(50);
         motor_OFF(); // desdoblar esta funcion tambien
         controlmotor=0;
         cuentareset=0;
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
