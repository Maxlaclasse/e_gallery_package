#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <stdio.h>
#include <string.h>
#include <WebSocketsClient.h>

//#define ssid      "Livebox-00C2"
//#define password  "PzpeMTpEkJ4bnQmJmG"

//#define ssid "iPhone de Maxime"
//#define password "maximerouillard"

#define ssid "NETGEAR_11g"
#define password "ESME94200"

#define SERVER_IP "192.168.1.4"
#define SERVER_PORT 8085

WebSocketsClient webSocket;

#define RXD2 16
#define TXD2 17
#define BaudLidar 230400

#define LED 2

#define brochePWMLidar  26 //Broche de la PWM du Lidar

#define POINT_PER_PACK 12
#define HEADER 0x54

int pwmChannelLidar = 1;
int LidarFrequency = 50000; //Fréquence d'utilisation du lidar
int PWMLidar = 4;
int resolution = 8;
int valueLidar;

int etatPWMLidar = 0; 


typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
  }LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
  }LiDARFrameTypeDef;



void setup() {

  Serial.begin ( 115200 );
  Serial2.begin(BaudLidar, SERIAL_8N1, RXD2, TXD2);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }  
  if(WiFi.status() == WL_CONNECTED){ 
    digitalWrite(LED, HIGH);
    }

  pinMode(brochePWMLidar, OUTPUT);
  ledcSetup(pwmChannelLidar, LidarFrequency, resolution);
  ledcAttachPin(PWMLidar, pwmChannelLidar);
  ledcWrite(pwmChannelLidar, 80);

  //config LEDs
  pinMode(LED,OUTPUT);

  // Start WebSocket client
  webSocket.begin(SERVER_IP, SERVER_PORT, "/");
  webSocket.onEvent(onWebSocketEvent);
  //webSocket.connect();

}

void onWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      break;
  } 
}

char* read_LiDAR_frame(LiDARFrameTypeDef *frame) {
  
  //char tab1[sizeof(LiDARFrameTypeDef) * 2 + 1];
  //char tab2[sizeof(LiDARFrameTypeDef) * 2 + 1];
  
  Serial.print("Header: ");
  Serial.println(frame->header);
  Serial.print("Version/Length: ");
  Serial.println(frame->ver_len);
  Serial.print("Speed: ");
  Serial.println(frame->speed);
  Serial.print("Start Angle: ");
  Serial.println(frame->start_angle);
  for (int i = 0; i < POINT_PER_PACK; i++) {
    Serial.print("Distance: ");
    Serial.print(frame->point[i].distance);
    Serial.print(", Intensity: ");
    Serial.println(frame->point[i].intensity);
  }
  Serial.print("End Angle: ");
  Serial.println(frame->end_angle);
  Serial.print("Timestamp: ");
  Serial.println(frame->timestamp);
  Serial.print("CRC8: ");
  Serial.println(frame->crc8);
  Serial.println();
  
}

void loop() {

    webSocket.loop();

    static bool frameStarted = false;
    static uint8_t buffer[sizeof(LiDARFrameTypeDef)];
    static int bufferIndex = 0;
    float angle_step;

    // Read a single byte from the UART
    int bytes_read = Serial2.readBytes((char*)&buffer[bufferIndex], 1);

    if (bytes_read == 1) {
      if (bufferIndex == 0 && buffer[bufferIndex] == 0x54) {
        // Start of a new frame
        frameStarted = true;
        bufferIndex++;
      } else if (frameStarted) {
        bufferIndex++;

        if (bufferIndex == sizeof(LiDARFrameTypeDef)) {
          // Complete frame received
          frameStarted = false;
          bufferIndex = 0;

          // Process the frame
          LiDARFrameTypeDef frame;
          char charbuffer[sizeof(LiDARFrameTypeDef)];
          memcpy(&frame, buffer, sizeof(frame));

          if (frame.header == 0x54 && frame.ver_len == 0x2C) {
            // Only process the frame if the header and ver_len match the expected value
          
              
              webSocket.sendBIN((uint8_t*)&frame, sizeof(frame)); 
              
              
                       
            //webSocket.sendBIN((uint8_t*)&frame, sizeof(frame));
            //read_LiDAR_frame(&frame);
            
          
          }
        }
      }

    }
}
