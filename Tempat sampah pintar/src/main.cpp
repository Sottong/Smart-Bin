#include <Arduino.h>

#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <Ultrasonic.h>
#include <LiquidCrystal_I2C.h>

#define RED_LED 5
#define YEL_LED 18
#define GRN_LED 19


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


/* millis */

unsigned long prev_interval_1 = 0;
const unsigned long interval_1 = 1000;
unsigned long prev_interval_2 = 0;
const unsigned long interval_2 = 1000;
unsigned long prev_interval_3 = 0;
const unsigned long interval_3 = 5000;


/* Ultrasonic */

Ultrasonic ultrasonic1(15, 2);	// An ultrasonic sensor HC-04
Ultrasonic ultrasonic2(14, 12);	// An ultrasonic sensor HC-04

int jarak_objek = 0;
int kapasitas = 0;

/* LCD I2C */

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


void setup() {
  Serial.begin(115200);
  Serial2.begin(GPSBaud); //GPS init

  // Menghubungkan ke jaringan Wi-Fi
  wifi_init();

  //read GPS
  gps_read();

  //LCD init
  lcd_setup();

  //LED init
  led_init();

}

void loop() {
  // Kode program utama di sini
  unsigned long current_time = millis();

  //periode 1 detik untuk baca sensor
  if (current_time - prev_interval_1 >= interval_1){

    prev_interval_1 = current_time; 

    // cek status koneksi wifi
    if (WiFi.status() != WL_CONNECTED) {
      fl_status_online = 0;
      Serial.println("Koneksi Wi-Fi terputus. Menghubungkan ulang...");
      reconnect_to_wifi(); // Menghubungkan ulang jika koneksi terputus
    }
    else fl_status_online = 1;
    

    //cek data GPS
    if(last_latitude != latitude ){
      gps_read();
      last_latitude = latitude;
      Serial.print("new latitude : ");
      Serial.println(latitude);
      Serial.print("last latitude : ");
      Serial.println(last_latitude);
    }
    
    jarak_objek = ultrasonic1.read(); // Prints the distance on the default unit (centimeters)
    kapasitas = ultrasonic2.read();   // Prints the distance on the default unit (centimeters)
    Serial.print("jarak : ");
    Serial.println(jarak_objek);   
    Serial.print("kapasitas : ");
    Serial.println(kapasitas);   
  }

  // periode 1 detik proses sensor
  if(current_time - prev_interval_2 >= interval_2){

    prev_interval_2 = current_time;

    //cek apakah ada orang akan buang sampah
    if(jarak_objek < 5 && kapasitas > 3){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Tutup Terbuka");
      lcd.setCursor(0,1);
      lcd.print("Silahkan buang");
    }
    else if(jarak_objek < 5 && kapasitas <= 3){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Sampah Penuh");
      lcd.setCursor(0,1);
      lcd.print("tutup terkunci");
    }
  }

  // periode 5 detik update LCD
  if(current_time - prev_interval_3 >= interval_3){

    prev_interval_3 = current_time;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Terpakai :");
    lcd.setCursor(11,0);
    lcd.print(kapasitas);

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
      lcd.print("Sedang");
    }
    else if(kapasitas >= 15){
      digitalWrite(RED_LED, LOW);
      digitalWrite(YEL_LED, LOW);
      digitalWrite(GRN_LED, HIGH);

      lcd.setCursor(0,1);
      lcd.print("Longgar");
    }




    

  }


  

}

/* Wifi */

//koneksi pertama

// void wifi_connected(){
//   Serial.println();
//   Serial.println("Terhubung ke jaringan WiFi");
//   Serial.print("Alamat IP: ");
//   Serial.println(WiFi.localIP());
// }

void wifi_init() {
  Serial.println("Menghubungkan ke WiFi...");
  WiFi.begin(ssid, password);
}

//koneksi ulang 
void reconnect_to_wifi(){
  WiFi.disconnect();
  delay(100);
  WiFi.begin();
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