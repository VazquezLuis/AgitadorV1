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
const int Boton = PF_0;  //PUSH2 
const int danger = PF_4; //PUSH1
const int umbral = 150; // umbral de valor analogico del pote para iniciar el motor
uint32_t seg=0; //segundos para el Timer de capa 8
uint32_t status=0; // estado de los pulsadores
uint16_t minutos_t=0; // minutos del temporizador
uint8_t minutos_s, horas_s; // minutos y horas para mostrar por LCD

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

void initbotones()
{
  GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
  GPIOIntRegister(GPIO_PORTF_BASE,BotonesIsr);
  GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
  GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
}

void BotonesIsr(){
  status = GPIOIntStatus(GPIO_PORTF_BASE,true);
 // Serial.println(status); //DEBUG
  
  if( (status & GPIO_INT_PIN_4) == GPIO_INT_PIN_4){
    delay(50);
    minutos_t++;
    if (minutos_t > 1440)minutos_t=0;
    Serial.print("Minutos: ");  
    Serial.println(minutos_t);
    } 
  
  if( (status & GPIO_INT_PIN_0) == GPIO_INT_PIN_0){
    delay(50);
    minutos_t--;
    if (minutos_t > 1440 && minutos_t <= 65535)minutos_t=1440;
    //if (
    Serial.print("Minutos: "); 
    Serial.println(minutos_t);
    } 

  GPIOIntClear(GPIO_PORTF_BASE,status);

}

void setup()
{
  Serial.begin(9600); //Comunicacion
  pinMode(RED_LED,OUTPUT); // Para la ISR del timer
  initTimer(); //inicializo el timer
  pinMode(Boton, INPUT_PULLUP);
  pinMode(danger, INPUT_PULLUP);
  pinMode(GREEN_LED,OUTPUT);
  pinMode(pot, INPUT);
  pinMode(PB_3,OUTPUT);
  initbotones();
 
}

void loop() 
{
  unsigned long ulPeriod;
  unsigned int Hz = 30;   // frequency in Hz  
  ulPeriod = (SysCtlClockGet() / Hz)/ 2;
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A,ulPeriod -1);
  uint16_t pwm;
  
  
  while (1)
  {
   // test();
    if ( analogRead(pot) > umbral ){
    pwm=pote();
    //analogWrite(PB_3,pwm);
    motor_ON(pwm);
    }
  //  temporizador(10); // programo el cartel 10 segundos
  /*
    if(!digitalRead(danger))
      digitalWrite(RED_LED,HIGH);
    else
      digitalWrite(RED_LED,LOW);
    */
    
 //   min_a_hs(minutos_t);
    
    if (!digitalRead(Boton)){ //si apreto el boton, prendo LED)
      digitalWrite(GREEN_LED,HIGH);
    }
    else{    //si no apreto, hago lo de siempre
    digitalWrite(GREEN_LED,LOW);
   // Serial.println(seg/60); 
    Serial.println(pwm); 
    delay(1000);
    }
  }
}

// funciones para usuario

//funcion de apagar y prender motor Indeptes

uint32_t pote(){ // el pote se conecta en PE_3
  uint32_t duty;
  if ( analogRead(pot) > umbral ) //umbral se seteara globalmente
    duty = analogRead(pot) / escala;
  return duty;
}

void motor_ON(uint32_t duty){
  
 if (duty >=245) duty=245; // Rango max 96% PWM
 if (duty <= 60) duty=60;  // Rango min 19,6% PWM
 
//  while(duty < 4000){ // valor final se cambia con las pruebas
   //duty = 30;
   analogWrite(motor,duty);
   Serial.println("Motor ON en PB_3"); 
  // duty++;  
  // }
 }

// Funciones relacionadas al Tiempo 
void temporizador (uint32_t tiempo1){
 
  if ( (seg / 60) == tiempo1){ // tiempo 1 en minutos
  //prender o apagar algo
  Serial.println("Prendemos o apagamos algo"); //debug
  seg = 0;}
}

void min_a_hs(uint16_t min){
	horas_s=min/60;
        minutos_s=min-(horas_s*60);
	Serial.print("Horas convertidas:");
	Serial.println(horas_s);
        Serial.print("Minutos convertidos:");
        Serial.println(minutos_s);
}

void test(){ // No Funciona - probar
uint32_t startPressed, endPressed, buttonState, lastButtonState;
uint32_t NoPresstime, Presstime;
buttonState = digitalRead(PUSH1);

  // button state changed
  if (buttonState != lastButtonState) {

      // the button was just pressed
      if (buttonState) {
          startPressed = millis();
          NoPresstime = startPressed - endPressed;
          Serial.print("Sin Presionar"); 
          Serial.println(NoPresstime); 
          

      // the button was just released
      } 
      if(!buttonState) {
          endPressed = millis();
          Presstime = endPressed - startPressed;
	  Serial.print("Presionado"); 
          Serial.println(Presstime); 
           }

  }

  // save the current state as the last state, 
  //for next time through the loop
  lastButtonState = buttonState;

}
