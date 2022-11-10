#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>

//------definiciones variables
#define ID "L00001"   //id del modulo
#define passwordAP "12345678"   //contraseña del acces point para conexion a wifi
#define mqtt_server "192.168.0.200"
const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
const uint16_t kRecvPin = 14; // ESP8266 GPIO pin to use. Recommended: 14 (D5).
//----------------------------
IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.

IRrecv irrecv(kRecvPin);

decode_results results;
#define fclk 80000000 //frecuencia del clock

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
#define TOPIC_BUFFER_SIZE  (50)
char topico[MSG_BUFFER_SIZE];

int t=0; //para aviso de presencia cada 3 s
int modo_operacion=0;   //para diferenciar el modo deteccion de codigo
                        //del modo de funcionamiento normal
int publicar=0;      //si llega el pedido de reconocimiento uso esta flag para habilitar
char *n_boton="xx",*ant_boton="xx"; 
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
  char topico_recibido[TOPIC_BUFFER_SIZE];
  
  snprintf (topico_recibido, TOPIC_BUFFER_SIZE, "/mod/%s/comandos", ID);
  if(!strcmp(topic,topico_recibido)){
    
    //payload : normal
    if((char)payload[0]=='n' && (char)payload[1]=='o' && (char)payload[2]=='r'){
      Serial.println("vuelvo a normal");
      modo_operacion=0;
    }
    //payload : configuracion
    if((char)payload[0]=='c' && (char)payload[1]=='o' && (char)payload[2]=='n'){
      Serial.println("entro a conf");
      modo_operacion=1;
    }
  }

  snprintf (topico_recibido, TOPIC_BUFFER_SIZE, "/mod/%s", ID);
  if(!strcmp(topic,topico_recibido)){
    if(modo_operacion){
      if((char)payload[0]=='b'){
        //agregar id del boton al string para enviar
        publicar=1;
      }
    } else {
      long codigo = atoi((char*)payload);
      Serial.println(codigo);
      //irsend.sendNEC(0xE0E0D02F);   //envio el codigo
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    String clientId = ID;
    if (client.connect(clientId.c_str())) {
      //si logro conectarse
      client.publish("/mod/status", ID);
      snprintf (topico, TOPIC_BUFFER_SIZE, "/mod/%s/#", ID);
      client.subscribe(topico,1);
    } else {
      //si no logro conectarse se clava 3 seg y vuelve a intentar en el loop
      delay(3000);
    }
  }
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

    //---init mqtt
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    //---init timer
    timerinit();

    //---init ledIR
    irsend.begin();
    irrecv.enableIRIn();  // Start the receiver
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if(modo_operacion && publicar){
    if (irrecv.decode(&results)) {
      // print() & println() can't handle printing long longs. (uint64_t)
      serialPrintUint64(results.value, HEX);
      Serial.println("");
      irrecv.resume();  // Receive the next value
      publicar=0;
      Serial.println("publico");
    }
    //publico id_boton+codigo
    
  }
  
  if(t>2){
    //cada 3 segundos publico un aviso de presencia
    client.publish("/status", ID);
    t=0;
  }
  
}
