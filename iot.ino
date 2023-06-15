
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

//DHT11
#include "DHT.h"
#define DHTTYPE DHT11
#define DHTPIN D3
DHT dht(DHTPIN, DHTTYPE);


#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "YJSANTY" // User ID
#define WLAN_PASS       "Github"          // Pass

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com" // Web Side
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME     "ajit_natarajan"
#define AIO_KEY          "aio_ehoU39vpcylb22qNSTtS6Td4Nv7S"
// User Name : ajit_natarajan
//  Password : 02101996

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish mqtt_humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish mqtt_temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish mqtt_ldr = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ldr");
Adafruit_MQTT_Publish mqtt_soil = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil");

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/button");

/*************************** Sketch Code ************************************/

void MQTT_connect();
#define LEDPIN D4
const int ADC_pin = A0;

struct {
  float temp = 0.00;
  float humi = 0;
  int   ldr  = 0;
  int   soil = 0;
} data;


void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

uint32_t x = 0;
int testVariable = 0;
void mqtt_code() {
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);
      if (0 == strcmp((char *)onoffbutton.lastread, "0")) {
        digitalWrite(LEDPIN, HIGH);
      }
      if (0 == strcmp((char *)onoffbutton.lastread, "1")) {
        digitalWrite(LEDPIN, LOW);
      }
    }
  }

  //Send Serial
  if (1) {
    for (int i = 0; i < 13; i++) Serial.print("-"); Serial.print("Data");
    for (int i = 0; i < 13; i++) Serial.print("-"); Serial.println();
    Serial.print("Humidity:"); Serial.println(data.humi);
    Serial.print("Temperature:"); Serial.println(data.temp);
    Serial.print("LDR:"); Serial.println(data.ldr);
    Serial.print("Soil:"); Serial.println(data.soil);
    for (int i = 0; i < 30; i++) Serial.print("-"); Serial.println();
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(data.humi);
  lcd.print("%");
  lcd.setCursor(8, 0);
  lcd.print(data.temp);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("L:");
  lcd.print(data.ldr);
  lcd.setCursor(8, 1);
  lcd.print("S:");
  lcd.print(data.soil); lcd.print("%");


  mqtt_humidity.publish(data.humi);    delay(1000);
  mqtt_temperature.publish(data.temp); delay(1000);
  mqtt_ldr.publish(data.ldr);          delay(1000);
  mqtt_soil.publish(data.soil);        delay(1000);

}

##define WLAN_SSID       "Abu king of Ajantha" // User ID
##define WLAN_PASS       "abuzar1344"          // Pass


void read_dht() {
  data.temp = dht.readTemperature();
  data.humi = dht.readHumidity();
}

void read_ldr() {
  data.ldr = digitalRead(D5);
}

void read_soil() {

  data.soil = map(analogRead(A0), 0, 1023, 100, 0);
}

void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println(); Serial.println();
  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Program Start");
  delay(2000);
  Serial.println("Program Started");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting...");
  Serial.print("Connecting to ");

  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  mqtt.subscribe(&onoffbutton);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected");
  pinMode(LEDPIN, OUTPUT); digitalWrite(LEDPIN, HIGH);


  dht.begin();
  pinMode(D5, INPUT);
  pinMode(A0, INPUT);
}

void loop() {
  mqtt_code();
  read_dht();
  read_soil();
  read_ldr();
  //delay(20000);
}