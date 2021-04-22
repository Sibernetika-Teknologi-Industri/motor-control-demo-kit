/*
 **************************************************
 *  File    : pid_position_control_dev_siber.ino
 *  Author  : Dimas AP., Husnul A., Prasetyo W.
 *            (PT Sibernetika Teknologi Industri)
 *  Brief   : Kode untuk PID Position Control Kit
 **************************************************
 */

/* Includes **********************************/
#include <Wire.h> // Library komunikasi I2C
#include <LiquidCrystal_I2C.h> // Library modul I2C LCD


/* Definition List **************/
// PID Param Definition
#define KP 1.8
#define KI 0.05
#define KD 3.0

// Pin Definition
#define ENC_A 2
#define ENC_B 3
#define MTR_CW 5
#define MTR_CCW 6
#define BT1 7
#define LM_SW1 8
#define LM_SW2 9
#define POT A0

// Limit Definition
#define MAX_ENC 506
#define MAX_PWM 100
#define MIN_PWM 30

/* Variable List ****************/
double err, err_I, err_D, err_prev, mtr_pwm_cmd;
volatile int mtr_pos;
int timer1_counter;
int read_pot, set_point, set_point_prev;
bool last_state = false;
float mtr_pos_last;

int i = 0;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

/* Function Promises ***********/
void count_encoder();
void check_set_point_change();
float wrapVal(float, float);
float wrapAngle(float, float);

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

  //Serial.begin(9600);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A),count_encoder,RISING);

  pinMode(MTR_CW, OUTPUT);
  pinMode(MTR_CCW, OUTPUT);

  pinMode(LM_SW1, INPUT_PULLUP);
  pinMode(LM_SW2, INPUT_PULLUP);
  pinMode(BT1, INPUT_PULLUP);

  mtr_pos = 0;

  // inisialisasi LCD:
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
  read_pot = analogRead(0);
  set_point = wrapVal(map(read_pot, 0, 1023, 0, MAX_ENC),MAX_ENC);
  check_set_point_change(); //refresh LCD only when set point change

  update_lcd();
  
}

/* Function Declaration ******************************/
void update_lcd(){
  lcd.setCursor(0, 0);
  lcd.print("PID:");
  lcd.setCursor(4, 0);lcd.print(" ");
  
  lcd.setCursor(5, 0);
  lcd.print(printPID(KP));
  
  lcd.setCursor(8, 0);lcd.print(" ");
  
//  lcd.setCursor(6, 0);
//  lcd.print("I:");
  lcd.setCursor(9, 0);
  lcd.print(printPID(KI));
//  lcd.setCursor(11, 0);
//  lcd.print("D:");

  lcd.setCursor(12, 0);lcd.print(" ");
  lcd.setCursor(13, 0);
  lcd.print(printPID(KD));
  // Set Point
  lcd.setCursor(0, 1);
  lcd.print("SP:");
  lcd.setCursor(3, 1);
  lcd.print(set_point_prev);
  lcd.setCursor(8, 1);
  lcd.print("ACT:");
  lcd.setCursor(12, 1);
  lcd.print(mtr_pos);
  
  if( (abs(mtr_pos_last)>100 && abs(mtr_pos) < 100) || (abs(mtr_pos_last)>10 && abs(mtr_pos) < 10) || (mtr_pos_last<00 && (mtr_pos) > 00)){
    lcd.setCursor(12,1);
    lcd.print("      ");
  }
  mtr_pos_last = mtr_pos;
}

// Fungsi untuk interrupt perhitungan encoder
void count_encoder() {
  if (digitalRead(ENC_B)) mtr_pos +=1;
  else mtr_pos -= 1;

//  mtr_pos = wrapVal(wrapAngle(mtr_pos, MAX_ENC),MAX_ENC);

//  //Serial.println(mtr_pos);
}

// Fungsi untuk update set_point ke LCD
void check_set_point_change() {
  if (abs(set_point - set_point_prev) > 10) //avoid stack number
  {
    lcd.setCursor(3,1);
    lcd.print("    ");
    set_point_prev = set_point;
  }
  else {
    set_point_prev = set_point;
  }
}

// Fungsi untuk PID (Interrupt service routine - tick every 0.1sec)
ISR(TIMER1_OVF_vect) {
  // Set timer
  TCNT1 = timer1_counter;

  // Count PID Output
  
  bool bt = digitalRead(BT1);

  if (bt != last_state) {
    last_state = bt;
    err = err_prev = err_I = err_D = mtr_pwm_cmd = 0;
  }

//  if(!last_state) {
//   
//
//  } else {
//     while(!digitalRead(LM_SW1)){ // remove ! on real implementation
//       digitalWrite(MTR_CW, LOW);
//       analogWrite(MTR_CCW, 100);
//     }
//     if(!digitalRead(LM_SW1)) mtr_pos = 0;
//     
//   }

  if(last_state) set_point = 0;
  
  err = set_point - mtr_pos;
  if(abs(err) < 1) err = 0;
  if(err/abs(err) != err_prev/abs(err_prev)) err_I = 0; // anti windup

  err_D = (err - err_prev);
  mtr_pwm_cmd = err * KP + err_I * KI + err_D * KD;
  err_prev = err;
  err_I += err;
    
  // Move the motor
  mtr_pwm_cmd = floor(wrapVal(mtr_pwm_cmd, MAX_PWM));
  if(mtr_pwm_cmd > 0) mtr_pwm_cmd = mtr_pwm_cmd/abs(mtr_pwm_cmd) * map(abs(mtr_pwm_cmd), 0, MAX_PWM, MIN_PWM, MAX_PWM);

  if (mtr_pwm_cmd > 0){
    digitalWrite(MTR_CW, LOW);
    analogWrite(MTR_CCW, abs(mtr_pwm_cmd));
  } else {
    digitalWrite(MTR_CCW, LOW);
    analogWrite(MTR_CW, abs(mtr_pwm_cmd));
  }

  //Serial.println(err);
}

// Utils
float wrapVal(float a, float b){
  if (a<-b) return -b;
  else if (a>b) return b;
  else return a;
}

float wrapAngle(float a, float lim){
  float x = a;
  if(a > lim) {
    while(x - lim > lim){
      x = x-lim;
    }
    return x;
  }
  else if(a < 0) return lim-a;
  else return a;

}

float printPID(float a){
  if(a<0.1) return a*10;
  else return a;
}
