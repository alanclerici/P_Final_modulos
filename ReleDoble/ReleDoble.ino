#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <WiFiUdp.h>  //udp

//------definiciones variables
#define ID1 "R00001"   //id del modulo
#define ID2 "R00002"
#define passwordAP "12345678"   //contraseña del acces point para conexion a wifi
//----------------------------

#define pinrele1 4
#define pinrele2 5
#define fclk 80000000 //frecuencia del clock
unsigned int localPort = 8888;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
#define TOPIC_BUFFER_SIZE  (50)
char topico[MSG_BUFFER_SIZE];

int t=0,pin; //para aviso de presencia cada 3 s. pin para usar en callback

// buffers for receiving and sending data (udp)
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,

WiFiUDP Udp;

void isr_timer(){
  t++;
}

void timerinit(){
  timer1_enable(TIM_DIV256,TIM_EDGE,TIM_LOOP);
  timer1_write(fclk/256); //1 segundo de periodo
  timer1_attachInterrupt(isr_timer);
}

//callback de recepcion de mensajes mqtt
void callback(char* topic, byte* payload, unsigned int length) {

  snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/comandos", ID1);
  if(!strcmp(topic,topico)){
    snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID1);
    pin=pinrele1;
  } else {
    snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID2);
    pin=pinrele2;
  }

  if((char)payload[0] =='g' && (char)payload[1] =='e' && (char)payload[2] =='t'){
    Serial.println("get");
    //snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID1);
    if(digitalRead(pin)){
      client.publish(topico, "on",2);   //el segundo parametro es lalongitud del msg
    } else {
      client.publish(topico, "off",3);
    }
  }
  if((char)payload[0] =='o' && (char)payload[1] =='n'){
    Serial.println("on");
    digitalWrite(pin,HIGH);
    //snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID1);
    client.publish(topico, "on",2);
  }
  if((char)payload[0] =='o' && (char)payload[1] =='f' && (char)payload[2] =='f'){
    Serial.println("off");
    digitalWrite(pin,LOW);
    //snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID1);
    client.publish(topico, "off",3);
  }
  if((char)payload[0] =='t' && (char)payload[1] =='o' && (char)payload[2] =='g'){
    Serial.println("toggle");
    digitalWrite(pin,!digitalRead(pin));
    //snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID1);
    if(digitalRead(pin)){
      client.publish(topico, "on",3);
    } else {
    client.publish(topico, "off",3);
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    String clientId = ID1;
    if (client.connect(clientId.c_str(),"modulo","modulo")) {
      //si logro conectarse
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/comandos", ID1);
      client.subscribe(topico,1);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/comandos", ID2);
      client.subscribe(topico,1);
    } else {
      //si no logro conectarse se clava 3 seg y vuelve a intentar en el loop
      delay(3000);
    }
  }
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

    //---init gpio
    pinMode(pinrele1,OUTPUT);
    digitalWrite(pinrele1,LOW);
    pinMode(pinrele2,OUTPUT);
    digitalWrite(pinrele2,LOW);

    //---wifi manager
    WiFiManager wm;
    bool res;
    //wm.resetSettings();
    wm.setConfigPortalTimeout(45);
    res = wm.autoConnect(ID1,passwordAP); // password protected ap. Bloqueante
    if(!res) {
        Serial.println("Failed to connect");
        ESP.restart();
    } 
    else {   
        Serial.println("connected");
    }

    //udp server init
    Udp.begin(localPort);
    int flag=1;
    long tf=millis();
    long ti=tf;
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

    //---init timer
    timerinit();
    
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if(t>2){
    //cada 3 segundos publico un aviso de presencia
    client.publish("/status", ID1);
    client.publish("/status", ID2);
    t=0;
  } 
}
