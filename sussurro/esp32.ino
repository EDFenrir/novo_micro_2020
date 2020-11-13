/*This code is used to ontrol the first microcontroller, the one that takes care of the sensors and the 
  PWM signal creation.   */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>

#include "ACS712.h"


#define OVERCURRENT 10

//data initialization regarding temperature sensor
OneWire pin(32);
DallasTemperature bus(&pin);
DeviceAddress sensor;


// the number of the LED pin
const int ledPin = 17;  // 16 corresponds to GPIO16

#define Potpin  34 // Defines Potpin as pin 34

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
int PotValue = 0;      
int PWMValue = 0;
int temp = 0;


//setting sensors properties
const int buttonPin = 4;              // the number of the pushbutton pin
const int pwmPin = 7;                 // the number of the LED pin
int pwmState = LOW;                   // the current state of the output pin
int buttonState;                      // the current reading from the input pin
int lastButtonState;            // the previous reading from the input pin
unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled
unsigned long delayPisca;
unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers
int pullupPin = 9;                    // the number of the pullup emulator, used to disable PWM        
int A1;
int reading;


// We have 30 amps version sensor connected to A1 pin of arduino
// Replace with your version if necessary
ACS712 sensor1(ACS712_30A, A1);

void setup(){
  Serial.begin(115200);
  buttonState = digitalRead(buttonPin);
  lastButtonState = buttonState;

  bus.begin();
  bus.getAddress(sensor, 0);
  
   // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  //Configure pins sensor of current
  pinMode(buttonPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(pullupPin, OUTPUT);
  digitalWrite(pwmPin,pwmState);
   
  // This method calibrates zero point of sensor of current,
  // It is not necessary, but may positively affect the accuracy
  // Ensure that no current flows through the sensor at this moment
  sensor1.calibrate();
  
  
  //coreTaskZero: used for the temperature sensor
  xTaskCreatePinnedToCore(
                    coreTaskZero,   // function that implements the task
                    "coreTaskZero", //task name
                    10000,          //number of words alocated for the task stack      
                    NULL,           //default input task
                    1,              //task priority
                    NULL,
                    0);       //task reference
                    
  delay(500); // delay to begin the next task

  //coreTaskOne: used for reading the potentiometer 
  xTaskCreatePinnedToCore(
                    coreTaskOne,   
                    "coreTaskOne", 
                    10000,      
                    NULL,       
                    3,          
                    NULL,
                    0);       

    delay(500); 
    
    //coreTaskThree: current sensor 
     xTaskCreatePinnedToCore(
                    coreTaskTwo,   
                    "coreTaskTwo", 
                    10000,      
                    NULL,       
                    2,          
                    NULL,
                    1);       

    delay(500); 


    xTaskCreatePinnedToCore(
                    coreTaskThree,   
                    "coreTaskThree", 
                    10000,      
                    NULL,       
                    2,          
                    NULL,
                    1);       

    delay(500); 


}

 
void loop(){
}


void coreTaskZero( void * pvParameters ){
    while(true){
    bus.requestTemperatures(); 
    temp = bus.getTempC(sensor); //read temperature
    vTaskDelay(500 / portTICK_PERIOD_MS);                      
    }
}

void coreTaskOne( void * pvParameters ){
    while(true){
    PotValue = analogRead(Potpin);// Make the reading of the ADC converter
    PWMValue = map(PotValue, 0, 4095, 0, 255); //map the potentiometer
    ledcWrite(ledChannel, PWMValue); //writes the PWM signal
    vTaskDelay(100 / portTICK_PERIOD_MS);  
    }
}


void coreTaskTwo( void * pvParameters ){
    while(true){
    reading = digitalRead(buttonPin);
    digitalWrite(pwmPin,pwmState);
    vTaskDelay(20 / portTICK_PERIOD_MS);  
    digitalWrite(pullupPin,!pwmState);
    
    if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
    if ((millis() - lastDebounceTime) > debounceDelay) {
     // whatever the reading is at, it's been there for longer than the debounce
     // delay, so take it as the actual current state:

    // if the button state has changed:
      if (reading != buttonState) {
       buttonState = reading;
    }
  }
  lastButtonState = reading;
  Serial.println(buttonState);
}
}


void coreTaskThree( void * pvParameters ){
    while(true){
      if (buttonState == HIGH){
      // set the LED:
      digitalWrite(pwmPin, pwmState);
    
      // save the reading. Next time through the loop, it'll be the lastButtonState:
      lastButtonState = reading;
    
      
      // Get current from sensor
      float I = sensor1.getCurrentAC();
      
      // Send it to serial
      Serial.println(String("I = ") + I + " A");
      
      // Wait one second before the new cycle
      vTaskDelay(50 / portTICK_PERIOD_MS);

      //Compares if the current read with the sensor is bigger than the overcurrent pre-defined
      if (I >= OVERCURRENT)
      //Set the pwmState at low in order to stop the pwm signal to flow     
        pwmState=LOW;
      else
      //Set the pwmState at high in order to allow the pwm signal to flow     
        pwmState=HIGH;   
  }else
      //if the deadman button is not pressed, then the pwm signal is blocked
        pwmState = LOW;
   }
}
