// ------ Program Setup ------
#include <ArduinoJson.h>
#define BUTTON_PIN 4

int alert = 0;
float targetBearing = 0;
int targetDistance = 0;
bool deviceConnected = false;

bool prevButtonState = HIGH;

// ------- Time Setup --------
#include <EEPROM.h>
#include <ESP32Time.h>

#define EEPROM_SIZE 4

int addr = 0;
ESP32Time rtc(28800); // GMT + 8

long readLongFromEEPROM(long address) {
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void writeLongIntoEEPROM(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
  EEPROM.commit();
}

// ---------------------------

// --- Magnetometer Setup ----

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1);

void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

float getCurrentBearing(void) {
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
//  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  Serial.print("Heading (degrees): "); Serial.println(heading * 180/M_PI);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
//  float declinationAngle = 0.22;
//  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Serial.println(headingDegrees);
  
  return headingDegrees;
}

// ---------------------------

// -------- BLE Setup --------
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_bt_device.h"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "d2769ce4-4941-41a8-87a8-7d8198a9ea85"
#define CHARACTERISTIC_UUID_RX "21b3d0b5-a458-480a-b063-629c7c1bad7b"
#define CHARACTERISTIC_UUID_TX "d6729370-102e-4eb3-a6e2-c1ac1fed26ff"

BLECharacteristic *wCharacteristic;
BLECharacteristic *nCharacteristic;

class HeadhomeServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class HeadhomeCharCallbacks: public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *wCharacteristic) {
    std::string rxValue = wCharacteristic->getValue();

    if (rxValue.length() > 0) {
      const char* receivedValue = rxValue.c_str();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, receivedValue);

      bool isAlertPacket = doc.containsKey("alert");

      if (isAlertPacket) {
        alert = doc["alert"];
      
        if (alert) {
          Serial.println("SOS detected...");
          targetBearing = doc["bearing"];
          targetDistance = doc["distance"];
        } 
      } else {
        long receivedTimestamp = doc["timestamp"];
        writeLongIntoEEPROM(addr, receivedTimestamp);
        rtc.setTime(receivedTimestamp);
      }
      
      Serial.println("**********");
      Serial.print("Received Data: ");
      Serial.println(receivedValue);
    }
  }
};

// ---------------------------

// ------- OLED Setup --------
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED displaybear width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define DEG2RAD 0.0174532925
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 'icons8-signal-24', 13x13px
const unsigned char epd_bitmap_icons8_signal_24 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x70, 0x07, 0x70, 0x07, 0x70, 0x07, 0x70, 0x77, 0x70, 
  0x77, 0x70, 0x77, 0x70, 0x77, 0x70, 0x00, 0x00, 0x00, 0x00
};
// 'icons8-no-connection-24', 13x13px
const unsigned char epd_bitmap_icons8_no_connection_24 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x50, 0x07, 0x50, 0x05, 0x50, 0x05, 0x50, 0x75, 0x50, 
  0x55, 0x50, 0x55, 0x50, 0x77, 0x70, 0x00, 0x00, 0x00, 0x00
};
// 'icons8-online-24', 13x13px
const unsigned char epd_bitmap_icons8_online_24 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x10, 0x40, 0x20, 0x20, 0x28, 0xa0, 0x5a, 0xd0, 0x57, 0x50, 0x5a, 0xd0, 
  0x28, 0xa0, 0x20, 0x20, 0x10, 0x40, 0x00, 0x00, 0x00, 0x00
};
// 'icons8-offline-24', 13x13px
const unsigned char epd_bitmap_icons8_offline_24 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x30, 0x20, 0x28, 0xa0, 0x5c, 0xd0, 0x52, 0x50, 0x59, 0x10, 
  0x28, 0x80, 0x20, 0x40, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00
};
// 'icons8-empty-battery-24', 13x13px
const unsigned char epd_bitmap_icons8_empty_battery_24 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x20, 0x10, 0x60, 0x10, 0x40, 0x10, 0x60, 0x10, 
  0x20, 0x10, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'icons8-full-battery-24', 13x13px
const unsigned char epd_bitmap_icons8_full_battery_24 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x3f, 0xf0, 0x7f, 0xf0, 0x7f, 0xf0, 
  0x7f, 0xf0, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'icons8-low-battery-24', 13x13px
const unsigned char epd_bitmap_icons8_low_battery_24 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x3f, 0xe0, 0x60, 0x70, 0x40, 0x70, 0x60, 0x70, 
  0x20, 0x70, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'arrow', 24x24px
const unsigned char epd_bitmap_arrow [] PROGMEM = {
  24, 24,
  B00000000, B00000000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00111100, B00000000, 
  B00000000, B11111110, B00000000, 
  B00000001, B11111111, B00000000, 
  B00000011, B10011011, B10000000, 
  B00000111, B00011000, B11000000, 
  B00000010, B00011000, B01000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00011000, B00000000, 
  B00000000, B00010000, B00000000,
};
// 'icons8-battery-level-24', 13x13px
const unsigned char epd_bitmap_icons8_battery_level_24 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x21, 0xf0, 0x61, 0xf0, 0x41, 0xf0, 0x61, 0xf0, 
  0x21, 0xf0, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'headhome-logo', 30x30px
const unsigned char epd_bitmap_headhome_logo [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x00, 
  0x00, 0x78, 0x78, 0x00, 0x00, 0xc0, 0x0c, 0x00, 0x01, 0x80, 0x06, 0x00, 0x01, 0x00, 0x02, 0x00, 
  0x03, 0x00, 0x03, 0x00, 0x02, 0x07, 0x81, 0x00, 0x02, 0x0f, 0xc1, 0x00, 0x06, 0x0f, 0xc1, 0x80, 
  0x06, 0x08, 0x40, 0x80, 0x06, 0x08, 0x40, 0x80, 0x02, 0x08, 0x41, 0x00, 0x02, 0x08, 0x41, 0x00, 
  0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x02, 0x00, 0x01, 0x80, 0x06, 0x00, 0x00, 0x80, 0x04, 0x00, 
  0x00, 0x40, 0x08, 0x00, 0x00, 0x60, 0x18, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x18, 0x60, 0x00, 
  0x00, 0x0c, 0xc0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 512)
const int epd_bitmap_allArray_LEN = 10;
const unsigned char* epd_bitmap_allArray[10] = {
  epd_bitmap_arrow,
  epd_bitmap_headhome_logo,
  epd_bitmap_icons8_battery_level_24,
  epd_bitmap_icons8_empty_battery_24,
  epd_bitmap_icons8_full_battery_24,
  epd_bitmap_icons8_low_battery_24,
  epd_bitmap_icons8_no_connection_24,
  epd_bitmap_icons8_offline_24,
  epd_bitmap_icons8_online_24,
  epd_bitmap_icons8_signal_24
};

void drawRotatedBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t angle) {

  uint8_t w = pgm_read_byte(bitmap++);
  uint8_t h = pgm_read_byte(bitmap++);

  int16_t newx, newy;
  uint8_t data = 0;

  float cosa = cos(angle * DEG2RAD), sina = sin(angle * DEG2RAD);

  x = x - ((w * cosa / 2) - (h * sina / 2));
  y = y - ((h * cosa / 2) + (w * sina / 2));

  for (int16_t j = 0; j < h; j++) {
    for (int16_t i = 0; i < w; i++ ) {
      if ((j * w + i) & 7) data <<= 1;
      else      data   = pgm_read_byte(bitmap++);

      newx = 0.5 + x + ((i * cosa) - (j * sina));
      newy = 0.5 + y + ((j * cosa) + (i * sina));

      if (data & 0x80) display.drawPixel(newx, newy, 1);
      //else            display.drawPixel(newx, newy, 0);
    }
  }
}

// ---------------------------

int test = 0;

void setup() {
  Serial.begin(115200);

  // OLED Setup
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  
  display.drawBitmap(47, 10, epd_bitmap_headhome_logo, 30, 30, WHITE);
  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(15,45);
  display.println("HeadHome");
  display.display();

  // Magnetometer Setup
  Serial.println("Setting up HMC5883 Magnetometer!");
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  displaySensorDetails();

  // BLE Setup
  Serial.println("Starting BLE work!");
  BLEDevice::init("HeadHome");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new HeadhomeServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  wCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  wCharacteristic->setCallbacks(new HeadhomeCharCallbacks());

  nCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_TX,
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  nCharacteristic->setValue("Hello from flutter!");
//  nCharacteristic->notify();
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  esp_ble_gap_set_device_name("HeadHome");
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

  // time setup
  EEPROM.begin(EEPROM_SIZE);
  long EEPROMtimestamp;
  EEPROMtimestamp = readLongFromEEPROM(addr);
  Serial.println("EEPROM timestamp: " + String(EEPROMtimestamp));
  
  if (!EEPROMtimestamp) {
    EEPROMtimestamp = 1678714513;
  }
  
  rtc.setTime(EEPROMtimestamp);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
//  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // checking for button press; sending SOS if button is pressed
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if(prevButtonState == HIGH && currentButtonState == LOW) {
    nCharacteristic->setValue("{\"SOS\":\"1\"}");
    nCharacteristic->notify();
    Serial.println("SOS Sent");
  }
  prevButtonState = currentButtonState;

  // print top bar icons
  display.clearDisplay();
  if (deviceConnected) {
    display.drawBitmap(0, 0, epd_bitmap_icons8_signal_24, 13, 13, WHITE);
  } else {
    display.drawBitmap(0, 0, epd_bitmap_icons8_no_connection_24, 13, 13, WHITE);
  }
  display.drawBitmap(15, 0, epd_bitmap_icons8_online_24, 13, 13, WHITE);
  display.drawBitmap(115, 0, epd_bitmap_icons8_full_battery_24, 13, 13, WHITE);

  // print top bar time
  display.setCursor(50, 5);
  display.setTextSize(1);
  display.print(rtc.getTime("%a %d"));

  // checking if wearable is in SOS mode; display arrow and distance if it is
  if (alert) {
    float currBearing = getCurrentBearing();    
    float bearingDelta = targetBearing - currBearing;

    // Ensuring that the delta bearing is within 0 and 360
    if(bearingDelta < 0)
      bearingDelta += 360;
    
    if(bearingDelta > 360)
      bearingDelta -= 360;

    // draw arrow and helper text
    drawRotatedBitmap(30, 40, epd_bitmap_arrow, bearingDelta);
    display.setTextSize(1);
    display.setCursor(50,25);

    if (bearingDelta < 45 || bearingDelta > 315) {
      display.print("Straight");
    } else if (bearingDelta > 225) {
      display.print("Turn Left");
    } else if (bearingDelta > 135) {
      display.print("Turn Back"); 
    } else {
      display.print("Turn Right");
    }
    
    display.setTextSize(2);
    display.setCursor(50,35);
    display.println(String(targetDistance) + "m");

  // display current time if wearable is idle
  } else {
    display.setCursor(15, 30);
    display.setTextSize(2);
    display.print(rtc.getTime());
  }
  display.display();
//  test++;
//  if (test > 360) {
//    test = 0;
//  }
  delay(1000);
}
