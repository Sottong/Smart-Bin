#include <Arduino.h>

#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <Ultrasonic.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>



#define RED_LED 5
#define YEL_LED 18
#define GRN_LED 19
#define SERVO_PIN 13

/* Blynk */
#define BLYNK_TEMPLATE_ID "TMPL6o7IBuD2I"
#define BLYNK_TEMPLATE_NAME "Tempat Sampah Pintar"


#define BLYNK_FIRMWARE_VERSION        "0.1.0"
#define BLYNK_PRINT Serial
#define APP_DEBUG
#define USE_ESP32_DEV_MODULE

// ConfigStore data;

#include "BlynkEdgent.h"


/* GPS */
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
// The TinyGPSPlus object
TinyGPSPlus gps;
// The serial connection to the GPS device
double latitude = 0;
double longitudde = 0;
double last_latitude = -1;

/* Wifi */

const char* ssid = "123"; // Ganti dengan nama WiFi Anda
const char* password = "innovate"; // Ganti dengan kata sandi WiFi Anda
bool fl_status_online = 0;

void wifi_init() ;
void reconnect_to_wifi();
void gps_read();
void gps_displayInfo();
void lcd_setup();
void led_init();
void servo_open();
void servo_close();
void myTimerEvent();

/* millis */

unsigned long prev_interval_1 = 0;
const unsigned long interval_1 = 1000;
unsigned long prev_interval_2 = 0;
unsigned long interval_2 = 10;
unsigned long prev_interval_3 = 0;
const unsigned long interval_3 = 5000;


/* Ultrasonic */

Ultrasonic ultrasonic1(15, 2);	// An ultrasonic sensor HC-04
Ultrasonic ultrasonic2(14, 12);	// An ultrasonic sensor HC-04

int jarak_objek = 0;
int kapasitas = 0;
int kapasitas_persen = 0;

/* LCD I2C */

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/* Servo */

Servo myservo;  // create servo object to control a servo
int pos = 0;
bool fl_servo_open = 0;

/* tombol */

bool fl_buka = 0;
bool fl_tutup = 0;

/* BLYNK*/

BlynkTimer timer;
ConfigStore data;
// WidgetMap myMap(V2);
#define UPLINK_INTERVAL 60000

BLYNK_WRITE(V3)
{
  // Set incoming value from pin V0 to a variable
  fl_buka  = param.asInt();
  
}

BLYNK_WRITE(V4)
{
  // Set incoming value from pin V0 to a variable
  fl_tutup  = param.asInt();
 
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPSBaud); //GPS init

  //blynk
  BlynkEdgent.begin();
  timer.setInterval(UPLINK_INTERVAL, myTimerEvent);

  // int index = 0;
  // double lat = 50.4495378;
  // double lon = 30.5251447;
  // double lat = latitude;
  // double lon = longitudde;
  
  
  // myMap.location(index, lat, lon, "Monument of Independence");
  

  // Menghubungkan ke jaringan Wi-Fi
  // wifi_init();

  //read GPS
  gps_read();

  //LCD init
  lcd_setup();

  //LED init
  led_init();

  //servo
  myservo.attach(SERVO_PIN);
  myservo.write(0);


}

void loop() {

  BlynkEdgent.run();
  if(fl_status_online) timer.run();



  // Kode program utama di sini
  unsigned long current_time = millis();

  //periode 1 detik untuk baca sensor
  if (current_time - prev_interval_1 >= interval_1){

    prev_interval_1 = current_time; 

    // cek status koneksi wifi
    if (WiFi.status() != WL_CONNECTED) {
      fl_status_online = 0;
      Serial.println("Koneksi Wi-Fi terputus. Menghubungkan ulang...");
      // WiFi.begin(configStore.wifiSSID, configStore.wifiPass);
      // reconnect_to_wifi(); // Menghubungkan ulang jika koneksi terputus
    }
    else fl_status_online = 1;
    

    //cek data GPS
    
      gps_read();
      Serial.print("new latitude : ");
      Serial.println(latitude);
      Serial.print("last latitude : ");

    
    
    jarak_objek = ultrasonic1.read(); // Prints the distance on the default unit (centimeters)
    if(!fl_servo_open)kapasitas = ultrasonic2.read();   // Prints the distance on the default unit (centimeters)
    if(kapasitas >= 28) kapasitas = 27;

    // kapasitas = 27;
    kapasitas_persen = map(kapasitas, 0, 27, 100, 0);

    Serial.print("jarak : ");
    Serial.println(jarak_objek);   
    Serial.print("kapasitas : ");
    Serial.println(kapasitas);   
    Serial.print("kapasitas persen : ");
    Serial.print(kapasitas_persen);  
    Serial.println("%");
  }

  // periode 1 detik proses sensor
  if(current_time - prev_interval_2 >= interval_2){

    prev_interval_2 = current_time;

    //cek apakah ada orang akan buang sampah
    if(jarak_objek < 10 && kapasitas > 3){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Tutup Terbuka");
      lcd.setCursor(0,1);
      lcd.print("Silahkan buang");
      servo_open();
      delay(5000);
      //tutup servo
      servo_close();
    }
    else if(jarak_objek < 8 && kapasitas <= 3){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Sampah Penuh");
      lcd.setCursor(0,1);
      lcd.print("tutup terkunci");
      delay(5000);
    }

    if(fl_buka){
      servo_open();
      fl_buka = 0;
    }
    if(fl_tutup){
      servo_close();
      fl_tutup = 0;
    }
  }

  // periode 5 detik update LCD
  if(current_time - prev_interval_3 >= interval_3){

    prev_interval_3 = current_time;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Terpakai :");
    lcd.setCursor(11,0);
    lcd.print(kapasitas_persen);
    lcd.setCursor(14,0);
    lcd.print("%");

    //status online (sementara pakai koneksi wifi, akan diganti dengan koneksi ke server)
    if(fl_status_online == 1){
      lcd.setCursor(9,1);
      lcd.print("Online");
    }
    else{
      lcd.setCursor(9,1);
      lcd.print("Offline");
    }

    //update LED
    if(kapasitas <= 3){
      digitalWrite(RED_LED, HIGH);
      digitalWrite(YEL_LED, LOW);
      digitalWrite(GRN_LED, LOW);

      lcd.setCursor(0,1);
      lcd.print("Penuh");

    }
    else if(kapasitas > 3 && kapasitas < 15){
      digitalWrite(RED_LED, LOW);
      digitalWrite(YEL_LED, HIGH);
      digitalWrite(GRN_LED, LOW);

      lcd.setCursor(0,1);
      lcd.print("Setengah");
    }
    else if(kapasitas >= 15){
      digitalWrite(RED_LED, LOW);
      digitalWrite(YEL_LED, LOW);
      digitalWrite(GRN_LED, HIGH);

      lcd.setCursor(0,1);
      lcd.print("Kosong");
    }

 

  }
  
}

/* GPS */

void gps_save_loc(){

  latitude = gps.location.lat();
  longitudde = gps.location.lng();
}
void gps_read(){

  while (Serial2.available() > 0)
  if (gps.encode(Serial2.read())) {
    gps_save_loc();
    gps_displayInfo();
  }
  // if (millis() > 5000 && gps.charsProcessed() < 10)
  // {
  //   Serial.println(F("No GPS detected: check wiring."));
  //   while(true);
  // }

  //catatn, perlu variable untuk simpan data latitude dan longitude
}

void gps_displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else 
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

/* LCD */

void lcd_setup(){
  lcd.init();  
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Smart Bin");
  lcd.setCursor(0,1);
  lcd.print("Ivan Alfianto");
}

/* LED */

void led_init(){
  pinMode(RED_LED, OUTPUT);
  pinMode(YEL_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
}

/* Servo */

void servo_open(){

  fl_servo_open = 0;
  for(int posDegrees = 0; posDegrees <= 70; posDegrees += 10) {
      myservo.write(posDegrees);
      Serial.println(posDegrees);
      delay(5);
  
  }

}

void servo_close(){
  if(fl_servo_open == 0){
    for(int posDegrees = 70; posDegrees >= 0; posDegrees -= 10) {
    myservo.write(posDegrees);
    Serial.println(posDegrees);
    delay(5);
    }
    fl_servo_open = 0;
  }
}

void myTimerEvent() // This loop defines what happens when timer is triggered
{
  // Serial.println("tes timer untuk uplink");

  //data yang dikirim 
  //kapasitas V0
  Blynk.virtualWrite(V0, kapasitas_persen);
  //status online device V1
  Blynk.virtualWrite(V1, fl_status_online);
  //GPS

  // double lat = 50.4495378;
  // double lon = 30.5251447;
  double lat = latitude;
  double lon = longitudde;
  Blynk.virtualWrite(V2, lon, lat);
  if(kapasitas_persen > 90){
    Blynk.logEvent("PENUH");
  }
}