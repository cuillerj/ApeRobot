void wakeUpNow()        // here the interrupt is handled after wakeup
{
  digitalWrite(yellowLed, 1);
}
void sleepNow()         // here we put the arduino to sleep
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     there is a list of sleep modes which explains which clocks and
     wake up sources are available in which sleep mode.

     In the avr/sleep.h file, the call names of these sleep modes are to be found:

     The 5 different modes are:
         SLEEP_MODE_IDLE         -the least power savings
         SLEEP_MODE_ADC
         SLEEP_MODE_PWR_SAVE
         SLEEP_MODE_STANDBY
         SLEEP_MODE_PWR_DOWN     -the most power savings

     For now, we want as much power savings as possible, so we
     choose the according
     sleep mode: SLEEP_MODE_PWR_DOWN

  */
  //  boolean led1 = digitalRead(13);
  boolean led1 = digitalRead(blueLed);
  boolean led2 = digitalRead(yellowLed);
  boolean led3 = digitalRead(greenLed);
  boolean led4 = digitalRead(redLed);
  digitalWrite(blueLed, 0);
  digitalWrite(yellowLed, 0);
  digitalWrite(greenLed, 0);
  digitalWrite(redLed, 0);
  delay(100);
  // set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  PRR = PRR | 0b00100000;
  set_sleep_mode(SLEEP_MODE_IDLE);   // sleep mode is set here
  power_adc_disable();  // analog digital converter
  power_spi_disable();  // bus SPI
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_timer3_disable();
  power_timer4_disable();
  power_usart0_disable();
  power_usart1_disable();
  //  power_usart3_disable();
  power_twi_disable(); // I2c

  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin

  /* Now it is time to enable an interrupt. We do it here so an
     accidentally pushed interrupt button doesn't interrupt
     our running program. if you want to be able to run
     interrupt code besides the sleep function, place it in
     setup() for example.

     In the function call attachInterrupt(A, B, C)
     A   can be either 0 or 1 for interrupts on pin 2 or 3.

     B   Name of a function you want to execute at interrupt for A.

     C   Trigger mode of the interrupt pin. can be:
                 LOW        a low level triggers
                 CHANGE     a change in level triggers
                 RISING     a rising edge of a level triggers
                 FALLING    a falling edge of a level triggers

     In all but the IDLE sleep modes only LOW can be used.
  */

  attachInterrupt(digitalPinToInterrupt(wakePin), wakeUpNow, LOW); // use interrupt  and run function
 
  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
  sleepRequest=false;
  // disable sleep...
  detachInterrupt(digitalPinToInterrupt(wakePin));      // disables interrupt 
  //  power_all_enable();
  //  power_usart1_disable();
  power_adc_enable();  // analog digital converter
  // power_spi_enable();  // bus SPI
  power_timer0_enable();
  power_timer1_enable();
  power_timer2_enable();
  power_timer3_enable();
  power_timer4_enable();
  power_usart0_enable();
  //  power_usart1_enable();
  PRR = PRR & 0b00000000;
  //digitalWrite(13, led1);
  digitalWrite(blueLed, led1);
  digitalWrite(yellowLed, led2);
  digitalWrite(greenLed, led3);
  digitalWrite(redLed, led4);

  // wakeUpNow code will not be executed
  // during normal running time.

}
