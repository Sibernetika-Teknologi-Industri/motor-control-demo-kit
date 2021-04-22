#include <PIDController.h>

// LCD
#include <Wire.h> // Library komunikasi I2C 
#include <LiquidCrystal_I2C.h> // Library modul I2C LCD
// default address 0x27
// tipe LCD 16x2 (16,2)
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
//
/* ENCODER_A and ENCODER_B pins are used to read the encoder
   data from the microcontroller, the data from the encoder
   comes very fast so these two pins have to be interrupt enabled
   pins
*/
#define ENCODER_A 2
#define ENCODER_B 3
/* the MOTOR_CW and MOTOR_CCW pins are used to drive the H-bridge
   the H-bridge then drives the motors, This two pins must have to
   be PWM enabled, otherwise the code will not work.
*/
#define MOTOR_CW 5
#define MOTOR_CCW 6
/*In this section we have defined the gain values for the
   proportional,integral, and derivative controller i have set
   the gain values with the help of trial and error methods.
*/
#define __Kp 200 // Proportional constant
#define __Ki 2 // Integral Constant
#define __Kd 2000 // Derivative Constant
volatile long int encoder_count = 0; // stores the current encoder count
volatile long int encoder_count_bef = 0; // stores the current encoder count
int integerValue = 0; // stores the incoming serial value. Max value is 65535
int integerValue_bef = 0;
int set_pot = 0;
char incomingByte; // parses and stores each individual character one by one
int motor_pwm_value = 255; // after PID computation data is stored in this variable.
PIDController pidcontroller;
void setup() {
  Serial.begin(115200); // Serial for Debugging
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  pinMode(MOTOR_CW, OUTPUT); // MOTOR_CW as Output
  pinMode(MOTOR_CCW, OUTPUT); // MOTOR_CW as Output
  /* attach an interrupt to pin ENCODER_A of the Arduino, and when the
      pulse is in the RISING edge called the function encoder().
  */
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
  pidcontroller.begin(); // initialize the PID instance
  pidcontroller.tune(260, 2.7, 2000); // Tune the PID, arguments: kP, kI, kD
  pidcontroller.limit(-255, 255); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!

  //button
  pinMode(7, INPUT_PULLUP);

  // inisialisasi LCD:
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("DC Motor PID");
  //  lcd.setCursor(0, 1);/
  //  lcd.print("www.ardutech/.com");
  delay(2000);
  lcd.clear();

}
void loop() {
  while (Serial.available() > 0) {
    integerValue = Serial.parseInt(); // stores the integerValue
    incomingByte = Serial.read(); // stores the /n character
    if (incomingByte == '\n') // if we receive a newline character we will continue in the loop
      Serial.println(integerValue);
    continue;
  }

  // Potensiometer Read
  set_pot = analogRead(0);
  integerValue = map(set_pot, 0, 1023, 0, 506);
  check_set_point_change(); //refresh LCD only when set point change
  
  if (!digitalRead(7)) {
    if (encoder_count < integerValue_bef) // if the motor_pwm_value is greater than zero we rotate the  motor in clockwise direction
    {
      motor_ccw(80);
      Serial.println("Running");
    }
    else // else we move it in a counter clockwise direction
    { 
      motor_cw(0);
      Serial.println("Running");
      //  Serial.println(encoder_count);// print the final encoder count.
    }
  }
  else {motor_ccw(0);}
  // LCD Display
  //  lcd.clear();/
  // PID
  lcd.setCursor(0, 0);
  lcd.print("P:");
  lcd.setCursor(2, 0);
  lcd.print(__Kp);
  lcd.setCursor(6, 0);
  lcd.print("I:");
  lcd.setCursor(8, 0);
  lcd.print(__Ki);
  lcd.setCursor(10, 0);
  lcd.print("D:");
  lcd.setCursor(12, 0);
  lcd.print(__Kd);
  // Set Point
  lcd.setCursor(0, 1);
  lcd.print("SP:");
  lcd.setCursor(3, 1);
  lcd.print(integerValue_bef);
  lcd.setCursor(7, 1);
  lcd.print("ACT:");
  lcd.setCursor(11, 1);
  lcd.print(encoder_count);
  //  delay(1000);/
  Serial.println(digitalRead(7));
}
void check_set_point_change() {
  if (abs(integerValue - integerValue_bef) > 10) //avoid stack number
  {
    lcd.clear();
    integerValue_bef = integerValue;
  }
  else {
    integerValue_bef = integerValue;
  }
}
void check_actual_enc_change() {
  if (abs(encoder_count - encoder_count_bef) > 10) //avoid stack number
  {
    lcd.clear();
    encoder_count_bef = encoder_count;
  }
  else {
    encoder_count_bef = encoder_count;
  }
}

void encoder() {
  if (digitalRead(ENCODER_B) == HIGH) // if ENCODER_B is high increase the count
    encoder_count++; // increment the count
  else // else decrease the count
    encoder_count--;  // decrement the count
}
void motor_cw(int power) {
  if (power > 0) {
    analogWrite(MOTOR_CW, power); //rotate the motor if the value is grater than 100
    digitalWrite(MOTOR_CCW, LOW); // make the other pin LOW
  }
  else {
    // both of the pins are set to low
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
  }
}
void motor_ccw(int power) {
  if (power > 0) {
    analogWrite(MOTOR_CCW, power);
    digitalWrite(MOTOR_CW, LOW);
  }
  else {
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
  }
}
