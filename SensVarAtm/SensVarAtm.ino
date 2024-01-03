#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include "DHT.h"


#define pinsw D7
//------definiciones variables
#define ID "S00001"   //id del modulo
#define passwordAP "12345678"   //contraseña del acces point para conexion a wifi
//----------------------------

unsigned int localPort = 8888;

// buffers for receiving and sending data (udp)
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,

WiFiUDP Udp;

#define fclk 80000000 //frecuencia del clock
#define DHTPIN D5     //pin D3 del esp
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bme; // I2C
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
#define TOPIC_BUFFER_SIZE  (50)
char topico[MSG_BUFFER_SIZE];

float newtemp=0, newpres=0, newhum=0;  //temperatura, presion y humedad
float anttemp=0, antpres=0, anthum=0;
int tbmp=0,tdht=0,tadp=0,tresetwifi=0;; //tbmp usada para bmp280, tdht para DHT11, tadp para aviso de presencia

//---wifi manager
WiFiManager wm;

float roundPunto5(float num){
  float aux=num-trunc(num);
  if (aux<0.33)
  {
    return (float)trunc(num);
  }
  else if (aux<0.66)
  {
    return (float)(trunc(num)+0.5);
  }
  else
  {
    return (float)(trunc(num)+1);
  }
}

void publicTempPres(){
    newtemp = roundPunto5(bme.readTemperature());
    newpres = round(bme.readPressure()/100);
    if(!isnan(newtemp) && anttemp!=newtemp){
      snprintf (msg, MSG_BUFFER_SIZE, "%3.1f", newtemp);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/temperatura", ID);
      client.publish(topico, msg,true);
      anttemp=newtemp;
    }
    if(!isnan(newpres) && antpres!=newpres){
      snprintf (msg, MSG_BUFFER_SIZE, "%d", (int)newpres);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/presion", ID);
      client.publish(topico, msg,true);
      antpres=newpres;
    }
}

void publicHum(){
  newhum = round(dht.readHumidity());
    if(!isnan(newhum)&& anthum!=newhum){
      snprintf (msg, MSG_BUFFER_SIZE, "%d", (int)newhum);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/humedad", ID);
      client.publish(topico, msg,true);
      anthum=newhum;
    }
}

//callback de recepcion de mensajes mqtt
void callback(char* topic, byte* payload, unsigned int length) {
  
  if ((char)payload[0] == 'g' && (char)payload[1] == 'e' && (char)payload[2] == 't') {
    publicTempPres();
    publicHum();
  }
  
}

void reconnect() {
  while (!client.connected()) {
    String clientId = ID;
    if (client.connect(clientId.c_str(),"modulo","modulo")) {
      //si logro conectarse, publica en el topico de status
      client.publish("/mod/status", ID);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/comandos", ID);
      client.subscribe(topico,1);
    } else {
      //si no logro conectarse se clava 3 seg y vuelve a intentar en el loop
      delay(3000);
    }
  }
}

void isr_timer(){
  tbmp++;
  tdht++;
  tadp++;
  tresetwifi++;
}

void timerinit(){
  timer1_enable(TIM_DIV256,TIM_EDGE,TIM_LOOP);
  timer1_write(fclk/256); //1 segundo de periodo
  timer1_attachInterrupt(isr_timer);
}

IPAddress getBroadcastIp(){
  String mask = WiFi.subnetMask().toString();
    String ip = WiFi.localIP().toString();

    char ip_arr[15];
    ip.toCharArray(ip_arr,15);
    char mask_arr[15];
    mask.toCharArray(mask_arr,15);
    int octeto[4];
    int contip=0,contmask=0;
    for(int i=0;i<4;i++){
      String auxip = String(ip_arr[contip]);
      String auxmask = String(mask_arr[contmask]);
      while(ip_arr[contip+1]!='.'){
        auxip=auxip+ip_arr[contip+1];
        contip++;
      }
      contip+=2;
      while(mask_arr[contmask+1]!='.'){
        auxmask=auxmask+mask_arr[contmask+1];
        contmask++;
      }
      contmask+=2;
      octeto[i]=auxip.toInt() & auxmask.toInt() | ~auxmask.toInt();
    }
    IPAddress ipred(octeto[0],octeto[1],octeto[2],octeto[3]);
    //Serial.println(ipred.toString());
    return ipred;
}

int listenUdp(){
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.printf("Received packet of size %d from %s:%d\n    (to %s:%d, free heap = %d B)\n",
                  packetSize,
                  Udp.remoteIP().toString().c_str(), Udp.remotePort(),
                  Udp.destinationIP().toString().c_str(), Udp.localPort(),
                  ESP.getFreeHeap());

    // read the packet into packetBufffer
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    return 0;
  }
return 1;
}

void sendBroadcast(){
  Udp.beginPacket(getBroadcastIp(), 2222);
  Udp.write("GetIp");
  Udp.endPacket();
}

void setup() {
    Serial.begin(115200);

    //---wifi manager
    WiFiManager wm;
    bool res;
    wm.setConfigPortalTimeout(45);
    res = wm.autoConnect(ID,passwordAP); // password protected ap
    if(!res) {
        Serial.println("Failed to connect");
        ESP.restart();
    } 
    else {   
        Serial.println("connected");
    }
    
    //---bmp280
    if (!bme.begin()) {  
      Serial.println("fallo bmp280");
    }

    //---dht11
    dht.begin();

    //udp server init
    Udp.begin(localPort);
    int flag=1;
    long ti=millis();
    long tf=ti;
    sendBroadcast();
    while(flag){
      tf=millis();
      if(tf-ti>1000){
        sendBroadcast();
        ti=tf;
      }
      flag = listenUdp();
    }

    //---init mqtt
    client.setServer(packetBuffer, 1883);
    client.setCallback(callback);
    
    //---inicializo timer
    timerinit();
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  if(tbmp){
    publicTempPres();
    tbmp=0;
  }

  if(tdht>1){
    publicHum();
    tdht=0;
  }

  if(tadp>2){
    //cada 3 segundos publico un aviso de presencia
    client.publish("/status", ID);
    tadp=0;
  }

  /*
  //reset conexion wifi
  if(!digitalRead(pinsw)){
    if(tresetwifi>3){
      wm.resetSettings();
      Serial.println("reseteao");
      //ESP.restart();
    }
  } else {
    tresetwifi=0;
  }
  */
}
