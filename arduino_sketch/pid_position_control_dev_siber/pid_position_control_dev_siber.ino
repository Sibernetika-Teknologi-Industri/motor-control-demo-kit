/*
 **************************************************
 *  File      : pid_position_control_dev_siber.ino
 *  Author    : Dimas AP., Husnul A., Prasetyo W.
 *              (PT Sibernetika Teknologi Industri - Bandung, Indonesia)
 *  Brief     : PID Position Control Kit
 *  Hardware  : 1. Arduino Nano
 *              2. HG7881 Motor Driver
 *              3. JGA25-370 motor (130 RPM)
 *                  - HALL Encoder (11 ticks/rev motor without gearbox)
 *                  - Gear Ratio 1:46 (506 ticks/rev output gearbox)
 *              4. 12 Volt Power Suppy
 *              5. 16x2 LCD display & I2C adapter
 *              6. Push Button
 *              7. Potentiometer
 *  Repo      : https://github.com/dispectra/motor-control-demo-kit/
 **************************************************
 */

/* Includes **********************************/
#include <Wire.h>               // I2C Communication Lib
#include <LiquidCrystal_I2C.h>  // LCD I2C Lib

/* Definition List **************/
/* PID Param Definition **
 * KP   : Proportional Gain
 * KI   : Integral Gain
 * KD   : Derivative Gain
 */
#define KP 0.7
#define KI 0.025
#define KD 0.5

/* Pin Definition **
 * ENC_A    : A pin of the encoder
 * ENC_B    : B pin of the encoder
 * MTR_CW   : Clockwise motor pin
 * MTR_CCW  : Counter-clockwise motor pin
 * BT1      : Button pin
 * POT      : Potentiometer pin
 */
#define ENC_A 2
#define ENC_B 3
#define MTR_CW 5
#define MTR_CCW 6
#define BT1 7
#define POT A0

/* Limit Definition **
 * MAX_ANALOG_INPUT : Max analogRead output (arduino nano has 10 bit analog input resolution, hence max value = 2^10-1)
 * MAX_ENC          : Ticks / Rev Hall Encoder
 * MAX_ANGLE        : Angle / Rev
 * MAX_PWM          : Max allowed PWM (arduino nano pwm pin has 8 bit resolution, hence max value = 2^8 - 1)
 * MIN_PWM          : Deadband before motor could start turning
 * SCALER           : Scaling increment to convert 1 tick of 506 ticks to x degree of 360 degs
 */
#define MAX_ANALOG_INPUT 1023
#define MAX_ENC 506
#define MAX_ANGLE 360
#define MAX_PWM 255.0f
#define MIN_PWM 45.0f
#define SCALER 0.71146f

/* Variable List ****************/
// PID Vars
double err, err_I, err_D, err_prev, mtr_pwm_cmd;

// Encoder Vars
volatile float mtr_pos = 0;
float mtr_pos_last;

// Button & Potentio Vars
int read_pot, read_pot_prev, set_point;
bool last_state = false;

// LCD Vars
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

// ISR Vars
int timer1_counter;

/* Function Promises ***********/
void count_encoder();
float wrapVal(float, float);
float sign(float);
float floor_abs(float);
float printPID(float);

/* SETUP **********************/
void setup() {
  noInterrupts();
  //--------------------------timer1 setup
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 59286;   // 65536-16MHz/256/10Hz (59286 for 0.1sec)
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  //--------------------------end time setup
  interrupts();

  Serial.begin(9600);       //  Enable serial at baud rate 9600
  pinMode(ENC_A, INPUT);    //  Set ENC_A pin as input
  pinMode(ENC_B, INPUT);    //  Set ENC_B pin as input
  attachInterrupt(digitalPinToInterrupt(ENC_A),count_encoder,RISING); // Attach Interrupt at pin ENC_A

  pinMode(MTR_CW, OUTPUT);  //  Set MTR_CW pin as output
  pinMode(MTR_CCW, OUTPUT); //  Set MTR_CCW pin as output

  pinMode(BT1, INPUT_PULLUP); //  Set ENC_B pin as input_pullup

  // LCD Init
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("DC Motor PID");
  delay(2000);
  lcd.clear();
}

/* Loop *********************************/
void loop() {
  // Read Potentiometer
  read_pot = wrapVal(map(analogRead(0), 0, MAX_ANALOG_INPUT, 0, MAX_ANGLE),MAX_ANGLE);

  // If button engaged -> Set set_point to potentiometer reading, otherwise set it to 0
  if(!digitalRead(BT1)) set_point = read_pot;
  else set_point = 0;

  // Print states to LCD
  update_lcd();

  // Print SP and PV to Serial Plotter
  Serial.print("set_point:");Serial.print(set_point);
  Serial.print(", ");
  Serial.print("actual_value:");Serial.println(int(mtr_pos));
}

/* Function Declaration ******************************/

/* FX update_lcd **
 * @brief: To update display text on the 16x2 LCD via LiquidCrystal_I2C lib
 * @return_value: None
 */
void update_lcd(){
  // ROW 1 :  PID
  lcd.setCursor(0, 0);lcd.print("PID:");
  lcd.setCursor(4, 0);lcd.print(" ");
  lcd.setCursor(5, 0);lcd.print(printPID(KP));
  lcd.setCursor(8, 0);lcd.print(" ");
  lcd.setCursor(9, 0);lcd.print(printPID(KI));
  lcd.setCursor(12, 0);lcd.print(" ");
  lcd.setCursor(13, 0);lcd.print(printPID(KD));

  // ROW 2  : Set Point
  lcd.setCursor(0, 1);lcd.print("SP:");
  if(should_clear_screen(read_pot,read_pot_prev)) { // Prevent Character buildup
    lcd.setCursor(3,1);
    lcd.print("    ");
  }
  lcd.setCursor(3, 1);lcd.print(read_pot);

  // ROW 2  : Actual Position (process variable)
  lcd.setCursor(9, 1);lcd.print("PV:");
  if(should_clear_screen(mtr_pos,mtr_pos_last)){  // Prevent Character buildup
    lcd.setCursor(12,1);
    lcd.print("      ");
  }
  lcd.setCursor(12, 1); lcd.print(int(sign(mtr_pos) * floor(abs(mtr_pos))));

  // Update last vars
  mtr_pos_last = mtr_pos;
  read_pot_prev = read_pot;
}

/* FX count_encoder **
 * @brief: To update motor pos value based on encoder A and B readings
 *          (Run as interrupt function for pin ENC_A)
 * @return_value: None
 */
void count_encoder() {
  if (digitalRead(ENC_B)) mtr_pos +=SCALER;
  else mtr_pos -= SCALER;
}

/* BOOL should_clear_screen **
 * @brief: To decide whether it is required to clear the screen before printing the LCD Values
 *          (Preventing charater bulking)
 * @logic: If current value has less charater than previous value
 *          Then clear the screen before printing
 * @input: a = current value; prev = previous value
 * @return_value: boolean; true = required to clear, false vice versa
 */
bool should_clear_screen(float a, float prev){
  if((abs(prev)>=100 && abs(a) < 100) || (abs(prev)>=10 && abs(a) < 10) || (prev<0 && (a) >= 0) || (a == 0 && prev != 0)) return true;
  else return false;
}

/* ISR Control Loop **
 * @brief: To run the control loop every 0.1 sec via Interrupt Service Routine
 *          (Using hardware timer as the timer)
 * @logic: Calculate err --> Calculate PID output --> Send output to motor driver
 * @return_value: none
 */
ISR(TIMER1_OVF_vect) {
  // Set timer
  TCNT1 = timer1_counter;

  // Decide if the state just recently changed
  //    If so --> reset PID values
  bool bt = digitalRead(BT1);
  if (bt != last_state) {
    last_state = bt;
    err = err_prev = err_I = err_D = mtr_pwm_cmd = 0;
  }

  // Update Error States
  err = (set_point - mtr_pos);
  err = floor_abs(err);
  if(abs(err) < 1) err = 0;                   // Assume err < 1 as already good enough
  if(sign(err) != sign(err_prev)) err_I = 0;  // Reset Integral value whenever err crossed zero (Anti Windup)
  err_D = (err - err_prev);

  // Count PID Output
  mtr_pwm_cmd = err * KP + err_I * KI + err_D * KD;

  // Update Error States
  err_prev = err;
  err_I += err;

  // Make sure pwm output consider all the system constraints
  mtr_pwm_cmd = wrapVal(floor_abs(mtr_pwm_cmd), MAX_PWM);
  if(mtr_pwm_cmd != 0) mtr_pwm_cmd = sign(mtr_pwm_cmd) * map(abs(mtr_pwm_cmd), 0, MAX_PWM, MIN_PWM, MAX_PWM);

  // Move the motor accordingly
  if (mtr_pwm_cmd > 0){
    digitalWrite(MTR_CW, LOW);
    analogWrite(MTR_CCW, abs(mtr_pwm_cmd));
  } else {
    digitalWrite(MTR_CCW, LOW);
    analogWrite(MTR_CW, abs(mtr_pwm_cmd));
  }
}

/* Utils Declaration ******************************/
/* FLOAT wrapVal **
 * @brief: To wrap values into certain limit
 * @input: a = value to process; b = limit to process
 * @return_value: float
 */
float wrapVal(float a, float b){
  if (a<-b) return -b;
  else if (a>b) return b;
  else return a;
}

/* FLOAT sign **
 * @brief: To obtain the sign of a number
 * @input: a = value to process;
 * @return_value: float; -1 for negative values, 1 for positive ones.
 */
float sign(float a) {
  if(a == 0) return 1;
  else return a/abs(a);
}

/* FLOAT floor_abs **
 * @brief: To floor number both with positive or negative values
 * @input: val = number to process
 * @return_value: float
 */
float floor_abs(float val){
  return sign(val) * floor(abs(val));
}

/* FLOAT printPID **
 * @brief: To format PID value so it will be printed nicely on the LCD :)
 * @input: a = value to process;
 * @return_value: float
 */
float printPID(float a){
  float x = a;
  while(x<0.1){
    x = x * 10;
  }
  return x;
}
