#include <SoftwareSerial.h>
#include <MicroGear.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#define D2 4
#define D3 0
SoftwareSerial NodeSerial(D2, D3); // RX | TX
const char* ssid     = "NEWSSON-HOME-2.4G";
const char* password = "6344634400";

#define APPID   "embed112"
#define KEY     "4igIzulPvmXetTr"
#define SECRET  "IjnJoy6rlABprNEhZBLzfKMAg"
#define ALIAS   "MCU8266"
#define TargetWeb "HTML_web"
#define FEEDID "embed112Feed"

WiFiClient client;
MicroGear microgear(client);

void setup() {
  // put your setup code here, to run once:
  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  Serial.begin(9600);
  Serial.println("Starting...");
  NodeSerial.begin(115200);
  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
       delay(250);
       Serial.print(".");
    }

    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    microgear.init(KEY,SECRET,ALIAS);
    microgear.connect(APPID);

}


void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) 
{
    Serial.print("Incoming message --> ");
    msg[msglen] = '\0';
    Serial.println((char *)msg);
}


void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) 
{
    Serial.println("Connected to NETPIE...");
    microgear.setAlias(ALIAS);
}


void loop() 
{
    if (microgear.connected())
    {
       microgear.loop();
       //Serial.println("connected");
    }
   else 
   {
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
   }
   if(NodeSerial.available()>=3){
      Serial.println("rec");
      Serial.println(NodeSerial.available());
      char currentQ = NodeSerial.read();
      char contact = NodeSerial.read();
      while(NodeSerial.available()> 0) NodeSerial.read();
      String jsonData;
      String data = "/" + String(currentQ) + "/" +String(contact) + "/";
      char msg[128];
      data.toCharArray(msg,data.length());
      Serial.println(msg);    
      microgear.chat(TargetWeb , msg);
      jsonData = "{\"contact\":";
      jsonData += String(contact);
      jsonData += ",\"currentQ\":";
      jsonData += String(currentQ);
      jsonData += "}";
      microgear.writeFeed(FEEDID,jsonData,"vieXGnCnRE2cRdelUBoRgIX5Qy2TdVgt");
       }
   delay(10);
}
