#include <WiFi.h>
#include <MQTT.h>

//Koding OLED --------------------------------------->
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char ssid[] = "XXXXXXXXXX";
const char pass[] = "XXXXXXXXXX";

WiFiClient net;
MQTTClient client;

const int kontakPins[] = {4, 5, 18};
const int kontakCount = sizeof(kontakPins) / sizeof(kontakPins[0]);

#define TRH_ID 0
#define ROTARY_ID 1

float temp = 0, humi = 0;

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(kontakPins[1], HIGH);
    delay(1000);
  }

  digitalWrite(kontakPins[3], HIGH);
  Serial.print("\nconnecting...");
  while (!client.connect("esp32", "public", "public")) {
    Serial.print(".");
    delay(1000);
  }

  if (client.connected()) {
    Serial.println("\nconnected!");
  } else {
    Serial.println("\nconnection failed, retrying...");
  }
}


void readXY_MD02() {
  uint8_t buff[] = {
    0x01, // Devices Address
    0x04, // Function code
    0x00, // Start Address HIGH
    0x01, // Start Address LOW
    0x00, // Quantity HIGH
    0x02, // Quantity LOW
    0x20, // CRC LOW
    0x0B  // CRC HIGH
  };

  Serial2.write(buff, sizeof(buff));
  Serial2.flush(); // wait send completed

  if (Serial2.find("\x01\x04")) {
    uint8_t n = Serial2.read();

    temp = ((uint16_t)(Serial2.read() << 8) | Serial2.read()) / 100.0; //devided by 100
    humi = ((uint16_t)(Serial2.read() << 8) | Serial2.read()) / 100.0; //devided by 100
  } 
}


typedef struct {
  byte deviceID;
  float value1;
  float value2;
} SENSOR;

QueueHandle_t queueSensor = xQueueCreate(8, sizeof(SENSOR));

void trh(void *ptParam) {

  Serial2.begin(9600);
  SENSOR shtSensor;
  shtSensor.deviceID = TRH_ID;

  while (1 ) {
    readXY_MD02();    
    char tempString[8];
    dtostrf(temp, 1, 2, tempString);  //Convert float to String    
    client.publish("rtos/iot/temperature", tempString);    
    
    char humiString[8];
    dtostrf(humi, 1, 2, humiString);    
    client.publish("rtos/iot/humidity", humiString);
        


    shtSensor.value1 = temp;
    shtSensor.value2 = humi;    

    // TickType_t timeOut = portMAX_DELAY;
    TickType_t timeOut = 2000;
    if (xQueueSend(queueSensor, &shtSensor, timeOut) != pdPASS) {
      Serial.println("SHT: Queue is full.");
    }

    vTaskDelay(1000);
  }

}

void rotary(void *ptParam) {
  // Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
  const int potPin = A0;

  // variable for storing the potentiometer value
  int adcValue = 0;

  SENSOR RSensor;
  RSensor.deviceID = ROTARY_ID;

  while (1 ) {
    adcValue = analogRead(potPin);    
    char xString[8];
    dtostrf(adcValue, 1, 2, xString);  //Convert float to String
    client.publish("rtos/iot/distance", xString);    

    RSensor.value1 = adcValue;
    RSensor.value2 = 0.0;
    
    // TickType_t timeOut = portMAX_DELAY;
    TickType_t timeOut = 2000;
    if (xQueueSend(queueSensor, &RSensor, timeOut) != pdPASS) {
      Serial.println("LDR: Queue is full.");
    }

    vTaskDelay(1000);
  }

}

void lcdTask(void *ptParam) {

  oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS); 

  oled.clearDisplay(); // clear display  
  oled.setTextSize(1);         // set text size
  oled.setTextColor(WHITE);    // set text color
  oled.setCursor(0, 2);       // set position to display (x,y)
  oled.println("MJ Mokhtar"); // set text
  oled.setCursor(0, 13);       // set position to display (x,y)
  oled.println("Monitoring Sensor"); // set text
  oled.display();


  SENSOR data;
  while (1) {
    //TickType_t timeOut = portMAX_DELAY;
    TickType_t timeOut = 2000;
    if (xQueueReceive(queueSensor, &data, timeOut) == pdPASS) {

      switch (data.deviceID) {
        case TRH_ID:          
          oled.setCursor(0, 24);       // set position to display (x,y)
          oled.println("T: " +  String(data.value1, 2) +" "+ char(247) +"C"); // set text
          oled.setCursor(0, 35);       // set position to display (x,y)
          oled.println("%RH : " + String(data.value2, 1) + " %"); // set text
          oled.display();
          oled.clearDisplay(); // clear display
          break;
          
        case ROTARY_ID:
          oled.setCursor(0, 46);       // set position to display (x,y)
          oled.println("X: " + String(data.value1) + " mm"); // set text          
          oled.display();
          oled.clearDisplay(); // clear display
          break;

        default:
          Serial.println("LCD: Unkown Device");
          break;
      }
    }  else {
      Serial.println("LCD: Message Queue is Empty");
    };

  oled.clearDisplay(); // clear display    
  vTaskDelay(2000);
  }
}

void mqttLoop(void *pvParameters) {
  for (;;) {
    if (!client.connected()) {
      connect();
    }

    if (client.connected()) {
      digitalWrite(kontakPins[2], HIGH);
      client.loop();
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Pendekatan non-blocking untuk memastikan keberlanjutan loop
  }
}

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < kontakCount; i++) {
    pinMode(kontakPins[i], OUTPUT);
  }

  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Rx, Tx
  Serial2.setTimeout(200);

  // start wifi and mqtt
  WiFi.begin(ssid, pass);
  client.begin("192.168.XX.XX", 1883, net);
  

  xTaskCreate(trh, "TRH", 1024 * 4, NULL, 1, NULL);
  xTaskCreate(rotary, "Roraty", 1024 * 4, NULL, 1, NULL);
  xTaskCreate(lcdTask, "lcd", 1024 * 8, NULL, 1, NULL);
  xTaskCreate(
      mqttLoop,
      "MQTTLoopTask",
      10000,
      NULL,
      1,
      NULL);

}


void loop() {}