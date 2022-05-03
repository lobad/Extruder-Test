#include <PID_v1.h>
#include <math.h>
#include <Wire.h> 
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

// Motor A connections
int enA = 3;
int in1 = 5;
int in2 = 4;

// LCD geometry
const int LCD_COLS = 20;
const int LCD_ROWS = 4;

#define ThermistorPin A0            //Analog Pin thermistor connected   
#define ThresholdPin A1             //Analog pin temp pot connected
#define HeaterPin 9                //digital PWM pin heater connected
#define MotorPin A2
double SetPoint, ActualTemp, Output, MotorSpeed;

//Specify the links and initial tuning parameters
PID myPID(&ActualTemp, &Output, &SetPoint,2,5,1, DIRECT);

//the time we give the sensor to calibrate (10-30 secs according to the datasheet)
int calibrationTime = 10; 

double Thermister(int RawADC) {
 double Temp;
 Temp = log(((10240000/RawADC) - 10000));
 Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
 return Temp;
}

void setup()
{
  {
  // Set motor pins to output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  }
  
  int status;
  status = lcd.begin(LCD_COLS, LCD_ROWS);
  if(status) // non zero status means it was unsuccesful
  {
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(status); // does not return
  }
  pinMode(ThresholdPin, INPUT);
  pinMode(HeaterPin, OUTPUT);
  Serial.begin(115200);
  lcd.init();   // initialize the lcd
  lcd.backlight();
  lcd.clear();
  //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
  lcd.setCursor(1,0);
  lcd.print("obad.XYZ  Extruder");
  delay(5000);
  lcd.clear();
  lcd.setCursor(1,1);
  lcd.print("obad.XYZ  Extruder");
  delay(2500);
  lcd.clear();
  lcd.setCursor(1,2);
  lcd.print("obad.XYZ  Extruder");
  delay(2500);
  lcd.clear();
  lcd.setCursor(1,3);
  lcd.print("obad.XYZ  Extruder");
  {
      Serial.print(".");
      delay(1000);
      }
    Serial.println(" done");
    Serial.println("SENSOR ACTIVE");
    delay(50);
  SetPoint = map(analogRead(ThresholdPin),0,1023,0,300);
  ActualTemp = double(Thermister(analogRead(ThermistorPin)));  
   
  lcd.clear();
  lcd.setCursor(1,0); //Start at character 2 on line 0
  lcd.print("T.Set - Temp - Spd");
  lcd.setCursor(1,1);
  lcd.print(SetPoint);
  lcd.setCursor(1,3);
  lcd.print("obad.XYZ  Extruder");
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() 
{
  SetPoint = map(analogRead(ThresholdPin),0,1023,0,300);
  ActualTemp = double(Thermister(analogRead(ThermistorPin)));
  myPID.Compute();
  analogWrite(HeaterPin,Output);
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(9,1);
  lcd.print(ActualTemp); 
  lcd.setCursor(1,1);
  lcd.print("    ");
  lcd.setCursor(1,1);
  lcd.print(SetPoint);
  
  Serial.print(SetPoint);  Serial.print("     ");
  Serial.println(ActualTemp); 
  delay(100);
  {
                                          // Turn on motors
  digitalWrite(in1,HIGH);
  digitalWrite(in2, LOW);
                                          // Accelerate to MotorSpeed
  int MotorSpeed = analogRead(MotorPin);  
  MotorSpeed = map(analogRead(MotorPin),0,1023,0,255);
  analogWrite(enA, MotorSpeed);
  
  // Update lcd with speed
  lcd.setCursor(16,1);
  lcd.print(MotorSpeed); 
  delay(100);
  }
}
