#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <string.h>

//------definiciones variables
#define ID "R00001"   //id del modulo
#define passwordAP "12345678"   //contraseña del acces point para conexion a wifi
#define mqtt_server "192.168.0.200"
//----------------------------

#define pinrele D3
#define fclk 80000000 //frecuencia del clock

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
#define TOPIC_BUFFER_SIZE  (50)
char topico[MSG_BUFFER_SIZE];

int t=0; //para aviso de presencia cada 3 s

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

  snprintf (topico, TOPIC_BUFFER_SIZE, "%s", (char*)payload);
  Serial.println(topico);

  if((char)payload[0] =='g' && (char)payload[1] =='e' && (char)payload[2] =='t'){
    Serial.println("get");
    snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID);
    if(digitalRead(pinrele)){
      client.publish(topico, "on",2);   //el segundo parametro es lalongitud del msg
    } else {
      client.publish(topico, "off",3);
    }
  }
  if((char)payload[0] =='o' && (char)payload[1] =='n'){
    Serial.println("on");
    digitalWrite(pinrele,HIGH);
    snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID);
    client.publish(topico, "on",2);
  }
  if((char)payload[0] =='o' && (char)payload[1] =='f' && (char)payload[2] =='f'){
    Serial.println("off");
    digitalWrite(pinrele,LOW);
    snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/estado", ID);
    client.publish(topico, "off",3);
  }
}

void reconnect() {
  while (!client.connected()) {
    String clientId = ID;
    if (client.connect(clientId.c_str())) {
      //si logro conectarse
      client.publish("/mod/status", ID);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/comandos", ID);
      client.subscribe(topico,1);
    } else {
      //si no logro conectarse se clava 3 seg y vuelve a intentar en el loop
      delay(3000);
    }
  }
}

void setup() {
    Serial.begin(115200);

    //---init gpio
    pinMode(pinrele,OUTPUT);
    digitalWrite(pinrele,LOW);

    //---wifi manager
    WiFiManager wm;
    bool res;
    res = wm.autoConnect(ID,passwordAP); // password protected ap
    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {   
        Serial.println("connected");
    }

    //---init mqtt
    client.setServer(mqtt_server, 1883);
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
    client.publish("/status", ID);
    t=0;
  }
  
}
