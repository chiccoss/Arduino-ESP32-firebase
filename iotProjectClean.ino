//#include <common.h>
#include <FirebaseESP32.h>
#include <FirebaseFS.h>
#include <Utils.h>
#include "seeed_bme680.h"
#include "pitches.h"
#include <Arduino.h>
#include <Tone32.h>
#include <PN532_HSU.h>
#include <PN532.h>

#define BUZZER_PIN 26
#define BUZZER_CHANNEL 0


#define RXD2 16
#define TXD2 17

PN532_HSU pn532hsu(Serial2);
PN532 nfc(pn532hsu);

byte openValue[] = "0";
byte textoTag[sizeof(openValue)];
boolean written = false;
String valid = ("45 FA 1E 2B") ;
String idnumber;

/* Firebase ESP32
  https://www.electroniclinic.com/
*/

#include <WiFi.h>
#include <ChainableLED.h>
//#include <FirebaseESP32.h>

#define SPEAKER 34

int BassTab[] = {1911, 1702, 1516, 1431, 1275, 1136, 1012}; //bass 1~7

#define FIREBASE_HOST "firebase link"
#define FIREBASE_AUTH "firebase token"
#define WIFI_SSID "enter here"
#define IIC_ADDR  uint8_t(0x76)
#define WIFI_PASSWORD "enter here"
#define SENSOR_PATH "/sensor"
#define NFC_PORT "uart"
#define AIR_QUALITY_PORT 36
#define SPEAKER_PORT 26
#define LEDS_CLOCK 15
#define LEDS_CLOCK2 33
#define NUM_LEDS  3

void postTemperature();
void postGas();
void postHumidity();
void postOpenkey(int openValue);
void postPressure();
void postLedValue(int ledId, int value);
void getLedsValues();
void postAllSensorsData();
void setLeds();
void getFirstLedStatus();

//Define FirebaseESP32 data object
FirebaseData firebaseData;
int Vresistor = 26;
int Vrdata = 0;
Seeed_BME680 bme680(IIC_ADDR); /* IIC PROTOCOL */
ChainableLED leds(LEDS_CLOCK, LEDS_CLOCK2, NUM_LEDS);

void setup(){
  initNfc();
  //pinInit();
  pinMode(Vresistor, INPUT);
  while (!bme680.init()) {
    Serial.println("bme680 init failed ! can't find device!");
    delay(10000);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  /*
    This option allows get and delete functions (PUT and DELETE HTTP requests) works for device connected behind the
    Firewall that allows only GET and POST requests.

    Firebase.enableClassicRequest(firebaseData, true);
  */
  delay(100);
  Serial.println("------------------------------------");
  Serial.println("Connected...");
}

void loop()
{
 loopForNfc();
 postAllSensorsData();
 getFirstLedStatus();
 getSecondLedStatus();
 getThirdLedStatus();

 getFirstLedColor();
 getSecondLedColor();
 getThirdLedColor();
}
 
void postAllSensorsData() {
  postTemperature();
  postGas();
  postHumidity();
  //postOpenkey(1);
  postPressure();
  //postLedValue(1, 200);
}

void setDataSensor() {
  Vrdata = analogRead(Vresistor);
  //int Sdata = map(Vrdata,0,4095,0,1000);
  //Serial.println(Sdata);
  delay(1000);
  FirebaseJson json;
  json.set("/data", Vrdata);
  Firebase.updateNode(firebaseData, "/Sensor", json);
  Vrdata++;
}


void postTemperature( ) {

  if (bme680.read_sensor_data()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("temperature ===>> ");
  Serial.print(bme680.sensor_result_value.temperature);
  Serial.println(" C");

  FirebaseJson json;
  json.set("temperature", bme680.sensor_result_value.temperature);
  Firebase.updateNode(firebaseData, SENSOR_PATH, json);
}

void postGas( ) {
  if (bme680.read_sensor_data()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("gas ===>> ");
  Serial.print(bme680.sensor_result_value.gas / 1000.0);
  Serial.println(" Kohms");

  FirebaseJson json;
  json.set("gas", bme680.sensor_result_value.gas / 1000.0);
  Firebase.updateNode(firebaseData, SENSOR_PATH, json);
}


void postHumidity( ) {
  if (bme680.read_sensor_data()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("humidity ===>> ");
  Serial.print(bme680.sensor_result_value.humidity);
  Serial.println(" %");
  FirebaseJson json;
  json.set("humidity", bme680.sensor_result_value.humidity);
  Firebase.updateNode(firebaseData, SENSOR_PATH, json);
}


void postOpenkey(int openValue) {
  FirebaseJson json;
  json.set("openkey", openValue);
  Firebase.updateNode(firebaseData, SENSOR_PATH, json);
}


void postPressure( ) {
  if (bme680.read_sensor_data()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("pressure ===>> ");
  Serial.print(bme680.sensor_result_value.pressure / 1000.0);
  Serial.println(" KPa");
  FirebaseJson json;
  json.set("pressure", bme680.sensor_result_value.pressure / 1000.0);
  Firebase.updateNode(firebaseData, SENSOR_PATH, json);
}

void postLedValue(int ledId, int value) {
  FirebaseJson json;
  // json.set("pressure", pressure);
  // Firebase.updateNode(firebaseData,SENSOR_PATH,json);
}

void getLedsValues() {

  if (Firebase.getInt(firebaseData, "/switchLed/1")) {
    if (firebaseData.dataType() == "int") {
      Serial.println("value first ");
      Serial.println(firebaseData.intData());
    }
  } else {
    Serial.println(firebaseData.errorReason());
  }
}

void getFirstLedStatus() {
    if (Firebase.getInt(firebaseData, "/switchLed/1")) {
      if (firebaseData.dataType() == "int") {
        if (firebaseData.intData() == 0)
          leds.setColorRGB(0, 0, 0, 0);
        else
          leds.setColorRGB(0, 255, 255, 255);
      }
    } else {
      Serial.println(firebaseData.errorReason());
    }
    delay(1000);
  
}



void getSecondLedStatus() {
    if (Firebase.getInt(firebaseData, "/switchLed/2")) {
      if (firebaseData.dataType() == "int") {
        if (firebaseData.intData() == 0)
          leds.setColorRGB(1, 0, 0, 0);
        else
          leds.setColorRGB(1, 255, 255, 255);
      }
    } else {
      Serial.println(firebaseData.errorReason());
    }
    delay(1000);
  
}


void getThirdLedStatus() {
    if (Firebase.getInt(firebaseData, "/switchLed/3")) {
      if (firebaseData.dataType() == "int") {
        if (firebaseData.intData() == 0)
          leds.setColorRGB(2, 0, 0, 0);
        else
          leds.setColorRGB(2, 255, 255, 255);
      }
    } else {
      Serial.println(firebaseData.errorReason());
    }
    delay(1000);
  
}




void getFirstLedColor() {
leds.setColorRGB(0, Firebase.getInt(firebaseData, "setLed/1/r") ? firebaseData.intData() : 0, Firebase.getInt(firebaseData, "setLed/1/g") ? firebaseData.intData() : 0, Firebase.getInt(firebaseData, "setLed/1/b") ? firebaseData.intData() : 0);
}
void getSecondLedColor() {
  leds.setColorRGB(1, Firebase.getInt(firebaseData, "setLed/2/r") ? firebaseData.intData() : 0, Firebase.getInt(firebaseData, "setLed/2/g") ? firebaseData.intData() : 0, Firebase.getInt(firebaseData, "setLed/2/b") ? firebaseData.intData() : 0);
}
void getThirdLedColor() {
  leds.setColorRGB(2, Firebase.getInt(firebaseData, "setLed/3/r") ? firebaseData.intData() : 0, Firebase.getInt(firebaseData, "setLed/3/g") ? firebaseData.intData() : 0, Firebase.getInt(firebaseData, "setLed/3/b") ? firebaseData.intData() : 0);
}


  void pinInit()
  {
  pinMode(SPEAKER, OUTPUT);
  digitalWrite(SPEAKER, LOW);
  }
  void sound(uint8_t note_index)
  {
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(SPEAKER, HIGH);
    delayMicroseconds(BassTab[note_index]);
    digitalWrite(SPEAKER, LOW);
    delayMicroseconds(BassTab[note_index]);
  }
  }

  void initNfc(){


   Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("Hello Reading From NFC!");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.'); 
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  // configure board to read RFID tags
  nfc.SAMConfig();
  uint8_t val = nfc.readGPIO();
  Serial.println("GPIO");
  Serial.println((val >> 24) & 0xFF, HEX);
  Serial.println("Waiting for an ISO14443A Card ...");

  }


  void loopForNfc(){
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: "); 
    Serial.print(uidLength, DEC); 
    Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    Serial.println("");
    String str = (char*)uid;
    char L = str.length();
    tone(BUZZER_PIN, NOTE_C4, 1000, BUZZER_CHANNEL);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
    if (uidLength == 4)
    {
      // We probably have a Mifare Classic card ...
      Serial.println("Seems to be a Mifare Classic card (4 byte UID)");

      // Now we need to try to authenticate it for read/write access
      // Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
      Serial.println("Trying to authenticate block 4 with default KEYA value");
      uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

      // Start with block 4 (the first block of sector 1) since sector 0
      // contains the manufacturer data and it's probably better just
      // to leave it alone unless you know what you're doing
      success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya);

      if (success)
      {
        Serial.println("Sector 1 (Blocks 4..7) has been authenticated");
        uint8_t temp[48];

        // If you want to write something to block 4 to test with, uncomment
        // the following line and this text should be read back in a minute
        // data = { 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0};
        // success = nfc.mifareclassic_WriteDataBlock (4, data);

        // Try to read the contents of block 4
        //        success = nfc.mifareclassic_ReadDataBlock(4, data);
        int index = 0;
        for (int i = 4; i < 7; i++) {
          uint8_t data[16];
          success = nfc.mifareclassic_ReadDataBlock(i, data);
          //          nfc.PrintHexChar(data, 16);

          for (int j = 0; j < 16; j++) {
            temp[index++] = data[j];
            //            index++;
          }
        }

        if (success)
        {
          // Data seems to have been read ... spit it out
          Serial.println("Reading Block 4:");
          nfc.PrintHexChar(temp, 48);
          Serial.println("");

          uint8_t nopol[10];
          uint8_t tanggal[4];
          uint8_t nip[18];
          uint8_t expired[4];
          uint8_t stMasuk;
          uint8_t kodeGate;
          uint8_t stKartu;

          //          copy nopol
          index = 0;
          for (int i = 0; i < 10; i++) {
            nopol[i] = temp[index++];
            //            index++;
          }
          //copy tanggal
          for (int i = 0; i < 4; i++) {
            tanggal[i] = temp[index++];
            //            index++;
          }
          stMasuk = temp[index++];
          kodeGate = temp[index++];
          //          copy nip
          for (int i = 0; i < 18; i++) {
            nip[i] = temp[index++];
          }
          //copy expired
          for (int i = 0; i < 4; i++) {
            expired[i] = temp[index++];
          }
          stKartu = temp[index];

          Serial.print("Nopol: ");
          nfc.PrintHexChar(nopol, 10);
          Serial.println();
          Serial.print("Tanggal(Date): " );
          nfc.PrintHexChar(tanggal, 4);
          Serial.println();
          Serial.print("Nip: (byte)" );
          nfc.PrintHexChar(nip, 18);
          Serial.println();
          Serial.print("Expired: " );
          nfc.PrintHexChar(expired, 4);
          Serial.println();
          Serial.print("Status Masuk: " );
          Serial.println(stMasuk);

          // Wait a bit before reading the card again
          delay(1000);
        }
        else
        {
          Serial.println("Ooops ... unable to read the requested block.  Try another key?");
        }
      }
      else
      {
        Serial.println("Ooops ... authentication failed: Try another key?");
      }
    }

    if (uidLength == 7)
    {
      // We probably have a Mifare Ultralight card ...
      Serial.println("Seems to be a Mifare Ultralight tag (7 byte UID)");

      // Try to read the first general-purpose user page (#4)
      Serial.println("Reading page 4");
      uint8_t data[32];
      success = nfc.mifareultralight_ReadPage (4, data);
      if (success)
      {
        // Data seems to have been read ... spit it out
        nfc.PrintHexChar(data, 4);
        Serial.println("");

        // Wait a bit before reading the card again
        delay(1000);
      }
      else
      {
        Serial.println("Ooops ... unable to read the requested page!?");
      }
    }
  }
}

byte colorConverterGetR(String hexValue)
{
unsigned long rgb = 0x6f56a3;
byte red, green, blue;
red = rgb >> 16 ;

green = (rgb & 0x00ff00) >> 8;

blue = (rgb & 0x0000ff);

rgb = 0;

rgb |= red <<16;
rgb |= blue <<8;
rgb |=green;
 Serial.println(red);
  Serial.println(green);
 Serial.println(blue);
return red;
}

byte colorConverterGetG(String hexValue)
{
unsigned long rgb = 0x6f56a3;
byte red, green, blue;
red = rgb >> 16 ;

green = (rgb & 0x00ff00) >> 8;

blue = (rgb & 0x0000ff);

rgb = 0;

rgb |= red <<16;
rgb |= blue <<8;
rgb |=green;
 Serial.println(red);
  Serial.println(green);
 Serial.println(blue);
return red;
}

byte colorConverterGetB(String hexValue)
{
unsigned long rgb = 0x6f56a3;
byte red, green, blue;
red = rgb >> 16 ;

green = (rgb & 0x00ff00) >> 8;

blue = (rgb & 0x0000ff);

rgb = 0;

rgb |= red <<16;
rgb |= blue <<8;
rgb |=green;
 Serial.println(red);
  Serial.println(green);
 Serial.println(blue);
return red;
}
  
