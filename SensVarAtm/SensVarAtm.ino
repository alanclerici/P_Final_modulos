#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include "DHT.h"

//------definiciones variables
#define ID "S00001"   //id del modulo
#define passwordAP "12345678"   //contraseña del acces point para conexion a wifi
#define mqtt_server "192.168.0.200"
//----------------------------

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

float temp=0, pres=0, hum=0;  //temperatura, presion y humedad
int tbmp=0,tdht=0,tadp=0; //tbmp usada para bmp280, tdht para DHT11, tadp para aviso de presencia

void publicTempPres(){
    temp = bme.readTemperature();
    pres = bme.readPressure()/100;
    if(!isnan(temp)){
      snprintf (msg, MSG_BUFFER_SIZE, "%3.1f", temp);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/temperatura", ID);
      client.publish(topico, msg,true);
    }
    if(!isnan(pres)){
      snprintf (msg, MSG_BUFFER_SIZE, "%4.2f", pres);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/presion", ID);
      client.publish(topico, msg,true);
    }
}

void publicHum(){
  hum = dht.readHumidity();
    if(!isnan(hum)){
      snprintf (msg, MSG_BUFFER_SIZE, "%d", (int)hum);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/humedad", ID);
      client.publish(topico, msg,true);
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
}

void timerinit(){
  timer1_enable(TIM_DIV256,TIM_EDGE,TIM_LOOP);
  timer1_write(fclk/256); //1 segundo de periodo
  timer1_attachInterrupt(isr_timer);
}

void setup() {
    Serial.begin(115200);

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
    
    //---bmp280
    if (!bme.begin()) {  
      Serial.println("fallo bmp280");
    }

    //---dht11
    dht.begin();

    //---inicializo timer
    timerinit();

    //---init mqtt
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
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
}
