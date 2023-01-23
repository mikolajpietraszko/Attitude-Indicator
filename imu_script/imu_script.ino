#include <Wire.h> //LLR_SW_01
#include <LiquidCrystal.h> //LLR_SW_02
#include <MPU6050.h> //LLR_SW_03
#include <dht_nonblocking.h> //LLR_SW_04
#include <KalmanFilter.h> //LLR_SW_05
#define DHT_SENSOR_TYPE DHT_TYPE_11
//defining MPU-6050
MPU6050 mpu;
KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);
unsigned long timer = 0;
float timeStep = 0.01;
float accPitch = 0;
float accRoll = 0;
const int MPU_addr=0x68;
//defining LCD Display pinout
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
//defining DHT SENSOR
static const int DHT_SENSOR_PIN = 8;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
int tempinterval = 2000; //time interval for temperature measurement
//defining RESET BUTTON
int BUTTON_PIN = 9;
int lastState = LOW;
const int RESET_TIME  = 5000;
int currentState;
unsigned long pressedtime = 0;
unsigned long releasedtime = 0;

int secinterval = 1000;//time interval of one second

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    //delay(500);
  }
  // Calibration of the mpu. The device have to be at rest for the calibration to take effect
  mpu.calibrateGyro();
  lcd.begin(16, 2); // set up the LCD's number of columns and rows:
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop()
{
  //LLR_SW_06
buttonReset();
static unsigned long measurement_timestamp = millis( );
float temperature;
float humidity;
int tempval;
int kalPitch=0;
int kalRoll = 0;
int data[3];
  //LLR_SW_07
imuFunction(data);  
  //LLR_SW_12
dht11Function(&temperature, &humidity);
data[2] = round(temperature);

  //LLR_SW_14
if( millis( ) - measurement_timestamp >= secinterval )//sends data both to the lcd and over the serial port in 1 sec interval
{
  //LLR_SW_15
  sendData(data);
  //LLR_SW_16
  lcdDisplay(data);
  measurement_timestamp = millis( );
}
}

void imuFunction(int data[]){ //LLR_SW_07
  int kalPitch, kalRoll;
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // Kalman filter 
  //LLR_SW_08
  //LLR_SW_09
  kalPitch = round(kalmanY.update(accPitch, gyr.YAxis)); 
  kalRoll = round(kalmanX.update(accRoll, gyr.XAxis));  
  
  //locking maximal and minimal values  LLR_SW_10
  if (kalPitch>90){kalPitch=90;}
  else if (kalPitch<-90){kalPitch=-90;}
  if (kalRoll>90) {kalRoll=90;}
  else if (kalRoll<-90) {kalRoll=-90;}
  //LLR_SW_11
  data[0] = kalPitch;
  data[1] = kalRoll;
}

void buttonReset(){ ////LLR_SW_06
  currentState = digitalRead(BUTTON_PIN); //read the state of the reset button
  if(lastState == LOW && currentState == HIGH){pressedtime = millis();}
  else if(lastState == HIGH && currentState == LOW) { // button is released
    releasedtime = millis();
    long pressDuration = releasedtime - pressedtime;
    if( pressDuration > RESET_TIME ){
    lcd.clear();
    delay(6000);
    }
  }
lastState = currentState;
}

void lcdDisplay(int data[]){  //LLR_SW_16
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print( "Pitch = " );
    lcd.print(data[0]);
    lcd.print( " deg" );
    lcd.setCursor(0, 1);
    lcd.print( "Roll = " );
    lcd.print(data[1]);
    lcd.print( "deg" );
}

void dht11Function(float *temperature, float *humidity){ //LLR_SW_12
  static unsigned long measurement_timestamp = millis( );
  //Measure tempereture once in 2 seconds
  if( millis( ) - measurement_timestamp >= tempinterval )
  {
    if( dht_sensor.measure( temperature, humidity) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }
}

void sendData(int data[]){  //LLR_SW_15
Serial.print(data[0]);
Serial.print(":");
Serial.print(data[1]);
Serial.print(":");
Serial.print(data[2]);
Serial.println();
  }
