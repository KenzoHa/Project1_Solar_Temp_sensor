/*
Sources:
CODE WITH BLUETOOTH -> asked ChatGPT to integrate my code (commented below) into the samplecode from: https://randomnerdtutorials.com/esp32-web-bluetooth/ (3/10/2024)
https://canvas.kdg.be/courses/49815/pages/coding-guidelines?module_item_id=1041685 (01/10/2024)
https://wiki.dfrobot.com/Terminal_sensor_adapter_V2_SKU_DFR0055 (25/09/2024)
https://docs.arduino.cc/built-in-examples/digital/Debounce/ (26/09/2024)
OpenAI (2024) - GPT 3.5 - https://chatgpt.com/ 
Anthropic AI (2024) - Claude 3.5 Sonnet - https://claude.ai/new
*/

#include <Arduino.h>
#include <OneWire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Pin declaration
#define RBG_RED 25
#define RBG_GREEN 26
#define RBG_BLUE 14
#define LED_GREEN 4
#define BUTTON 12
#define LIGHT_SENSOR 39
#define DS18S20_PIN 13

// BLE variables
BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

unsigned long previousMillisBlinkLedGreen = 0; 
const long INTERVAL_BLINK_LED_GREEN = 250; 
unsigned long previousDebounceTime = 0;
const long DEBOUNCE_DELAY = 200;
unsigned long previousMillisLight = 0;
const long INTERVAL_LIGHT = 10000; 
unsigned long previousMillisTemp = 0; 
const long INTERVAL_TEMP = 10000;

// New variable for non-blocking timing
unsigned long previousMillisDataSend = 0;
const long INTERVAL_DATA_SEND = 10000; // 10 seconds

// Temperature sensor
OneWire ds(DS18S20_PIN);

// BLE UUIDs
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"   
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"   

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    std::string ledvalue = pLedCharacteristic->getValue(); 
    String value = String(ledvalue.c_str());
    if (value.length() > 0) {
      int receivedValue = static_cast<int>(value[0]);
      digitalWrite(LED_GREEN, receivedValue == 1 ? HIGH : LOW);
    }
  }
};

void blinkLedGreen() {
  unsigned long currentMillisBlinkLedGreen = millis(); 
  if(currentMillisBlinkLedGreen - previousMillisBlinkLedGreen >= INTERVAL_BLINK_LED_GREEN){
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    previousMillisBlinkLedGreen = currentMillisBlinkLedGreen;
  }
}

float getTemp() {
  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28) {
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // Start conversion

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  ds.reset_search();
  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB);
  return tempRead / 16;
}

int getLightVal() {
  return analogRead(LIGHT_SENSOR);
}

void showTempSerial() {
  unsigned long currentMillisTemp = millis(); 
  float temperature = getTemp();
  if(currentMillisTemp - previousMillisTemp >= INTERVAL_TEMP) {
    Serial.print("Temperature: "); 
    Serial.print(temperature); 
    Serial.println("°C");
    previousMillisTemp = currentMillisTemp;
  }
}

void showLightSerial() {
  unsigned long currentMillisLight = millis(); 
  int lightVal = getLightVal();
  if(currentMillisLight - previousMillisLight >= INTERVAL_LIGHT) {
    Serial.print("LDR-value: "); 
    Serial.println(lightVal);
    previousMillisLight = currentMillisLight;
  }
}

void pressButton() {
  int buttonState = digitalRead(BUTTON);
  unsigned long currentDebounceTime = millis();
  float temperature = getTemp();
  int lightVal = getLightVal();

  if(buttonState == HIGH && (currentDebounceTime - previousDebounceTime) >= DEBOUNCE_DELAY) {
    Serial.println("You pressed the button! ");
    Serial.print("Temperature: "); 
    Serial.print(temperature); 
    Serial.println("°C");
    Serial.print("LDR-value: "); 
    Serial.println(lightVal);
    previousDebounceTime = currentDebounceTime;
  }
}

void setColorRbg(int red, int blue, int green) {
  analogWrite(RBG_RED, red);
  analogWrite(RBG_BLUE, blue); 
  analogWrite(RBG_GREEN, green);
}

void controlRbg() {
  int lightVal = getLightVal();
  float temperature = getTemp();
  
  if(temperature <= 14.0) {
    setColorRbg(255, 0, 255); // Blue
  } else if(temperature >= 25.0) {
    setColorRbg(0, 255, 255); // Red
  } else if (temperature >= 14.1 && temperature <= 24.9) {
    if(lightVal <= 1800) {
      setColorRbg(0, 0, 255); // Purple
    } else {
      setColorRbg(0, 255, 0); // Yellow
    }
  }
}

// New function to show data when connected
void showDataConnected() {
  if (deviceConnected) {
    blinkLedGreen();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisDataSend >= INTERVAL_DATA_SEND) {
      float temperature = getTemp();
      int lightVal = getLightVal();
      
      String sensorData = "Temp: " + String(temperature) + ", Light: " + String(lightVal);
      pSensorCharacteristic->setValue(sensorData.c_str());
      pSensorCharacteristic->notify();
      
      Serial.print("Sensor data notified: ");
      Serial.println(sensorData);

      previousMillisDataSend = currentMillis; // Update the last time data was sent
    }
  }
}

// New function to handle connection status
void handleDeviceConnection() {
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start advertising");
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(RBG_RED, OUTPUT);
  pinMode(RBG_BLUE, OUTPUT);
  pinMode(RBG_GREEN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(LIGHT_SENSOR, INPUT);
  setColorRbg(255, 255, 255); // Turn off RGB

  // Initialize BLE
  BLEDevice::init("ESP32_Kenzo");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pSensorCharacteristic = pService->createCharacteristic(
                          SENSOR_CHARACTERISTIC_UUID,
                          BLECharacteristic::PROPERTY_READ |
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  Serial.println("Waiting for client connection...");
}

void loop() {
  //showTempSerial();       This can be used to test hardware while not connected
  //showLightSerial();      This can be used to test hardware while not connected
  controlRbg();
  pressButton();
  
  showDataConnected();      
  handleDeviceConnection(); // Call to handle connection status
}

//---------------------------------------------------------------------------------------------------------------------

// /*
// CODE WITHOUT BLUETOOTH !!
// Sources:
// https://canvas.kdg.be/courses/49815/pages/coding-guidelines?module_item_id=1041685 (01/10/2024)
// https://wiki.dfrobot.com/Terminal_sensor_adapter_V2_SKU_DFR0055 (25/09/2024)
// https://docs.arduino.cc/built-in-examples/digital/Debounce/ (26/09/2024)
// OpenAI (2024) - GPT 3.5 - https://chatgpt.com/ 
// Anthropic AI (2024) - Claude 3.5 Sonnet - https://claude.ai/new
// */

// //Libraries: 
// #include <Arduino.h>
// #include <OneWire.h>

// //Pin declaration
// //RBG -----> WE USE RBG instead of RGB
// #define RBG_RED 25
// #define RBG_GREEN 26
// #define RBG_BLUE 14
// //LEDGREEN
// #define LED_GREEN 4
// unsigned long previousMillisBlinkLedGreen = 0; 
// const long INTERVAL_BLINK_LED_GREEN = 250; 
// //PUSHBUTTON
// #define BUTTON 12
// unsigned long previousDebounceTime = 0;
// const long DEBOUNCE_DELAY = 200;
// //LDR / SOLARSENSOR
// #define LIGHT_SENSOR 39                    //testing -> 4095 = maxValue and 0 = minValue
// unsigned long previousMillisLight = 0;
// const long INTERVAL_LIGHT = 10000; 
// //TEMPSENSOR
// #define DS18S20_PIN 13 //DS18S20 Signal pin on digital 9
// OneWire ds(DS18S20_PIN);  //Temperature chip i/o on digital pin 9
// unsigned long previousMillisTemp = 0; 
// const long INTERVAL_TEMP = 10000;

// //put function declarations here:
// void blinkLedGreen() {                                             //will later be used during connecting with Bluetooth/WiFi/... 
//   unsigned long currentMillisBlinkLedGreen = millis(); 
//   if(currentMillisBlinkLedGreen - previousMillisBlinkLedGreen >= INTERVAL_BLINK_LED_GREEN){
//     if(digitalRead(LED_GREEN) == LOW){
//       digitalWrite(LED_GREEN, HIGH);
//       } 
//       else{
//         digitalWrite(LED_GREEN, LOW);
//         }
    
//     previousMillisBlinkLedGreen = currentMillisBlinkLedGreen;
//   }
// }
// float getTemp() {                                                  //returns the temperature from one DS18S20 in DEG Celsius (partially copied Sample Code)
//   byte data[12];
//   byte addr[8];

//   if ( !ds.search(addr)) {
//       //no more sensors on chain, reset search
//       Serial.println("no more sensors on chain, reset search!");
//       ds.reset_search();
//       return -1000;
//   }

//   if ( OneWire::crc8( addr, 7) != addr[7]) {
//       Serial.println("CRC is not valid!");
//       return -1000;
//   }

//   if ( addr[0] != 0x10 && addr[0] != 0x28) {
//       Serial.print("Device is not recognized");
//       return -1000;
//   }

//   ds.reset();
//   ds.select(addr);
//   ds.write(0x44,1); // start conversion, with parasite power on at the end

//   byte present = ds.reset();
//   ds.select(addr);
//   ds.write(0xBE); // Read Scratchpad

//   for (int i = 0; i < 9; i++) { // we need 9 bytes
//     data[i] = ds.read();
//   }

//   ds.reset_search();

//   byte MSB = data[1];
//   byte LSB = data[0];

//   float tempRead = ((MSB << 8) | LSB); //using two's compliment
//   float TemperatureSum = tempRead / 16;

//   return TemperatureSum;

// }
// int getLightVal() {                                                //gives the lightValue of the LDRsensor
//   return analogRead(LIGHT_SENSOR);
// }
// void showTempSerial() {                                            //every 10 sec the temperature is shown on serial monitor
//   unsigned long currentMillisTemp = millis(); 
//   float temperature = getTemp();
//   if(currentMillisTemp - previousMillisTemp >= INTERVAL_TEMP){
//     Serial.print("Temperature: "); Serial.print(temperature); Serial.println("°C");
//     previousMillisTemp = currentMillisTemp;
//   }
// }
// void showLightSerial() {                                           //every 10 sec the LDR's lightValue is shown on serial monitor
//   unsigned long currentMillisLight = millis(); 
//   int LightVal = getLightVal();
//   if(currentMillisLight - previousMillisLight >= INTERVAL_LIGHT){
//     Serial.print("LDR-value: "); Serial.println(LightVal);
//     previousMillisLight = currentMillisLight;
//   }
// }
// void pressButton() {                                               //When the button is pressed, both TempVal and LightVal will show on serial monitor
//   int buttonState = digitalRead(BUTTON);
//   unsigned long currentDebounceTime = millis();
//   float temperature = getTemp();
//   int lightVal = getLightVal();

//   if(buttonState == HIGH && (currentDebounceTime - previousDebounceTime) >= DEBOUNCE_DELAY){
//     Serial.println("You pressed the button! ");
//     Serial.print("Temperature: "); Serial.print(temperature); Serial.println("°C");
//     Serial.print("LDR-value: "); Serial.println(lightVal);
//     previousDebounceTime = currentDebounceTime;   //update debounce time
//   }
// }
// void setColorRbg(int red, int blue, int green) {                   //function to set RBG-colors. (Common Anode -> 0 = on / 255 = off)
//   analogWrite(RBG_RED, red);
//   analogWrite(RBG_BLUE, blue); 
//   analogWrite(RBG_GREEN, green);
// }
// void controlRbg() {                                                //turns RBG red when too hot, and turns RBG blue when too cold + //turns the RBG purple when not enough light, and yellow when enough light
//   int lightVal = getLightVal();
//   float temperature = getTemp();
//   if(temperature <= 14.0){                                 //if too cold 
//     setColorRbg(255, 0, 255);                              //blue light
//     // Serial.println("blue");
//   }
//   else if(temperature >= 25.0){                            //else if too hot 
//     setColorRbg(0, 255, 255);                              //red light
//     // Serial.println("red");
//   }
//   else if (temperature >= 14.1 && temperature <= 24.9){
//     if(lightVal <= 1800){                                  //if not enough (sun)light
//       setColorRbg(0, 0, 255);                              //rgb purple 
//       // Serial.println("purple");
//     }
//     else if(lightVal >=1801){                              //if enough (sun)light
//       setColorRbg(0, 255, 0);                              //rgb yellow 
//       // Serial.println("yellow");
//     }
//   }
// }

// void setup() {                                                    //put your setup code here, to run once:
//   Serial.begin(9600);
//   pinMode(RBG_RED, OUTPUT);
//   pinMode(RBG_BLUE, OUTPUT);
//   pinMode(RBG_GREEN, OUTPUT);
//   pinMode(LED_GREEN, OUTPUT);
//   pinMode(BUTTON, INPUT);
//   pinMode(LIGHT_SENSOR, INPUT);
//   setColorRbg(255, 255, 255); //turn off rbg 
// }

// void loop() {                                                     //put your main code here, to run repeatedly:
//   blinkLedGreen();
//   showTempSerial(); 
//   showLightSerial();                                            
//   controlRbg();
//   pressButton();  
// }
