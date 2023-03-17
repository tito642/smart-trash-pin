#include <Servo.h> 

Servo myservo;
const int servo_pin = 2;

#define trigPin 3            //sensor A
#define echoPin 4         //sensor A
int pos = 0;




#define btrigPin 7          //sensor B
#define bechoPin 8   

 

#include <Wire.h> 
 // #include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

#define LCD_I2C_ADDRESS 0x3f 
#define LCD_DISP_COLS   16
#define LCD_DISP_ROWS   2

#include <MQUnifiedsensor.h>

#define         Board                   ("Arduino NANO")

#define         Pin135                   (A2)  //Analog input 2 of your arduino


//LiquidCrystal_I2C lcd( LCD_I2C_ADDRESS, LCD_DISP_COLS, LCD_DISP_ROWS );

#define         RatioMQ135CleanAir        (3.6) //RS / R0 = 10 ppm 
#define         ADC_Bit_Resolution        (10) // 10 bit ADC 
#define         Voltage_Resolution        (5) // Volt resolution to calc the voltage
#define         Type                      ("Arduino NANO") //Board used

MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin135, Type);



float average ;
void setup() 
{
  Serial.begin(9600);

  myservo.attach(servo_pin);
  myservo.write(pos);


 pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
 
  pinMode(btrigPin, OUTPUT);
  pinMode(bechoPin, INPUT);

  
  lcd.init();
  lcd.backlight();
                 
 
 

  MQ135.init();
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setR0(9.03);
 
} 

void loop() {

int duration, distance;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;







   
   if (distance< 70)
  {
    
      for (pos = 0; pos <= 180; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
 
    delay(5000);
   
     for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
     


  
int bduration, bdistance;
  digitalWrite(btrigPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(btrigPin, LOW);
  bduration = pulseIn(bechoPin, HIGH);
  bdistance = (bduration/2) / 29.1;
  

  
  
 int val = bdistance ;
  val = map(val,41, 10, 0,100);
 
  lcd.clear();
delay(70);
lcd.setCursor (0,0);
lcd.print("average ");
lcd.print(val);
lcd.setCursor (13,0);
lcd.print("%");
delay(1000);


  MQ135.update(); 

MQ135.setA(110.47); MQ135.setB(-2.862); //CO2 
float CO2 = MQ135.readSensor(); 
  
  MQ135.setA(44.947); MQ135.setB(-3.445); // Toluene
float Toluene = MQ135.readSensor(); 
  
  MQ135.setA(102.2 ); MQ135.setB(-2.473); //NH4 
float NH4 = MQ135.readSensor(); 
  
  MQ135.setA(34.668); MQ135.setB(-3.369); //Acetone
float Acetone = MQ135.readSensor(); 


  lcd.clear();
delay(70);
lcd.setCursor (0,0);
lcd.print("Toluene ");
lcd.print(Toluene);
lcd.setCursor (13,0);
lcd.print("ppm");
lcd.setCursor (0,1);
lcd.print("NH4     ");
lcd.print(NH4);
lcd.setCursor (13,1);
lcd.print("ppm");
delay(2000);
lcd.clear();
delay(70);
lcd.setCursor (0,0);
lcd.print("Acetone ");
lcd.print(Acetone);
lcd.setCursor (13,0);
lcd.print("ppm");
lcd.setCursor (0,1);
lcd.print("CO2     ");
lcd.print(CO2);
lcd.setCursor (13,1);
lcd.print("ppm");
delay(2000);}}
