#include <stdint.h>
#include <stdbool.h>

#define PART_TM4C123GH6PM
#include "inc/tm4c123gh6pm.h"

/*****************************
 * LM4F120 - timer based blink 
 * Using TimerIntRegister
 * 80 Mhz clock
 *****************************/

#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

const int Boton = PUSH2;
unsigned int seg=0; //segundos para el Timer de capa 8

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
  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Clear the timer interrupt
  seg++; // si estamos a 30Hz - cuenta 1 seg
  //digitalWrite(RED_LED, digitalRead(RED_LED) ^ 1);              // Toggle led rojo - Control
}



void setup()
{
  Serial.begin(9600); //Comunicacion
  //pinMode(RED_LED,OUTPUT); // Para la ISR del timer
  initTimer(); //inicializo el timer
  pinMode(Boton, INPUT_PULLUP);
  pinMode(GREEN_LED,OUTPUT);
}

void loop() 
{
  unsigned long ulPeriod;
  unsigned int Hz = 30;   // frequency in Hz  
  ulPeriod = (SysCtlClockGet() / Hz)/ 2;
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A,ulPeriod -1);
  
  while (1)
  {
    temporizador(10); // programo el cartel 10 segundos
  
    if (!digitalRead(Boton)){ //si apreto el boton, prendo LED)
      digitalWrite(GREEN_LED,HIGH);
    }
    else{    //si no apreto, hago lo de siempre
    digitalWrite(GREEN_LED,LOW);
    Serial.println(seg/60); 
    delay(1000);
    }
  }
}

// funciones para usuario

//funcion de apagar y prender motor Indeptes

void temporizador (uint8_t tiempo1){
 
  if ( (seg / 60) == tiempo1){ // tiempo 1 en minutos
  //prender o apagar algo
  Serial.println("Prendemos o apagamos algo");
  seg = 0;}
}
