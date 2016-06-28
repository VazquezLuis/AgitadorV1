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


void initTimer()
{  // se habilita despues de 5 ciclos de reloj, calcular para no escribirlo antes)
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //habilito el timer0
  ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);   // 32 bits Timer
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0Isr);    // Registering  isr       
  ROM_TimerEnable(TIMER0_BASE, TIMER_A); 
  ROM_IntEnable(INT_TIMER0A); 
  ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  
}

void Timer0Isr(void)
{
  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Clear the timer interrupt
  digitalWrite(RED_LED, digitalRead(RED_LED) ^ 1);              // toggle LED pin
}

void setup()
{
  Serial.begin(9600);
  pinMode(RED_LED,OUTPUT);
  initTimer();
}

void loop()
{  
  unsigned long ulPeriod;
  unsigned int Hz = 5;   // frequency in Hz  
  ulPeriod = (SysCtlClockGet() / Hz)/ 2;
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A,ulPeriod -1);
  while (1)
  {
    Serial.println("While timer is running ..."); 
    delay(1000);
  }
}
