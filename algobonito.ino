#include <WiFi.h>
#include "ThingSpeak.h"

const char* ssid = "Tec-IoT";   // your network SSID (name) 
const char* password = "spotless.magnetic.bridge";   // your network password

WiFiClient  client;

unsigned long myChannelNumber = 1;
const char * myWriteAPIKey = "LE4VKE1LVSEK7ARF";
const char * myReadAPIKey  = "3ARL9SJHBUQ607AZ";

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

int read_field = 1;
int write_field = 2;

void setup()
{
    Serial.begin(115200);  //Initialize serial
  
    WiFi.mode(WIFI_STA);   

    randomSeed(analogRead(0));
  
    ThingSpeak.begin(client);  // Initialize ThingSpeak
}

void loop()
{  
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED)
  {
      Serial.print("Attempting to connect");
      while(WiFi.status() != WL_CONNECTED)
      {
          WiFi.begin(ssid, password); 
          delay(5000);     
      } 
      Serial.println("\nConnected.");
  }

  float dcb = random(30, 90); 
  if(dcb >= 0 and (millis() - lastTime) > timerDelay){
    write(dcb);
    lastTime = millis();
    Serial.println("DCB:" + String(dcb));
  }
  delay(1000);
}


void write(float data){
  error_msg(ThingSpeak.writeField(myChannelNumber, write_field, data, myWriteAPIKey), "write");
}

float read(){
  float res = ThingSpeak.readFloatField(myChannelNumber, read_field, myReadAPIKey);
  int e = ThingSpeak.getLastReadStatus();
  error_msg(e, "read");
  return e == 200 ? res : 0 ;
}

void error_msg(int e, char * type){
  if(e == 200)
  {
      Serial.print("Successful!!! > ");
      Serial.println(type);
  }
  else
  {
      Serial.print("Problem. HTTP error code " + String(e) + " > ");
      Serial.println(type);
  }
}
