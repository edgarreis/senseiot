
/*
  PowerBoard - Library Power Board Node V1
  Based on Emon.cpp - Library for openenergymonitor
  Based on Software Timer - vidadesilicio/software-timer-esp-8266
  Based on ArduinoOTA - github esp8266/ArduinoOTA
  Based on pubsubclient - github knolleary/pubsubclient
  Created by Edgar dos Reis, Guilherme Teixeira, November 05 2017
  GNU GPL
  modified to integrate OTA and MQTT functions
  by eng.edgar.reis@gmail.com November 12 2017
  
*/

#include <ESP8266WiFi.h>            // WiFi Library
#include <PubSubClient.h>           // MQTT Library
#include <ESP8266mDNS.h>            // DNS Library
#include <WiFiUdp.h>                // UDP Library
#include <ArduinoOTA.h>             // OTA Library
#include <EEPROM.h>                 // EEPROM Library
#include <Stream.h>                 // Stream Library
#include <user_interface.h>         //Biblioteca necessaria para acessar os Timer`s.
#include <Bounce2.h>                // Debounce Library
#include "string.h"                 // String Library
#include "EmonPower.h"              // Emon modified Library

#define RELE1 0  // D3
#define RELE2 14 // D5
#define REPORT_INTERVAL 1           // in sec

#define ADC_BITS    10
#define ADC_COUNTS  (1<<ADC_BITS)

// Wifi SSID e Password
const char* ssid = "";     
const char* password = "";  

// Tópicos MQTT
char* hellotopic   = "clients"; 
char* topicCurrent = "dev/node1/power";
char* topicRele1   = "cmd/node1/rele1";     //"dev/node1/rele1";
char* topicRele2   = "cmd/node1/rele2";     //"dev/node1/rele1";
char* outTopic     = "clients";
char* mqtt_server = "";
int port = 1883;
String clientId;

// Variaveis Power
String payload = "";
String SVrms;
String SIrms;
String SrealPower;
String SpowerFactor;
float  Irms;
float  Vrms;
float  realPower;
float  powerFactor;

// Instantiate EnergyMonitor
EnergyMonitor emon1;

// MQTT Client
String clientName;
WiFiClient wifiClient;

// Escopo da Função callback
void callback(char* topic, byte* payload, unsigned int length);

// Instantiate PubSubClient
PubSubClient client(mqtt_server, port, callback, wifiClient);

// Variaveis Temporarias
long lastMsg = 0;
char msg[50];
int value = 0;
int cnt;
long now;

// Variaveis p/ calculo dos tempos
unsigned long ant = 0;
unsigned long fim = 0;
unsigned long initial = 0;
unsigned long espera = 0;
unsigned long proc = 0;

// Variavel armazenada na RAM para Status. 
volatile byte status;

// Cria o Timer. Maximo de 7 Timer's.
os_timer_t tmr0;


void setup() {

  // Configura o Tirmer 0
  configTimer();
  // Configuração Wifi
  configWifi(ssid,password);
  // Configuração MQTT
  client.connect(clientId.c_str(), outTopic, 2, 1, "ESPClient Desconnected");
  client.setCallback(callback);
  
  // Multiplex Pin Define
  pinMode(MUX, OUTPUT);
  
  // Rele Pin Define
  pinMode(RELE1, OUTPUT);
  digitalWrite(RELE1, LOW);

  // EnergyMonitor Config e Calibração
  emon1.current(A0, 19);
  emon1.voltage(A0, 142, 2.3);

  // Connect to MQTT Brocker
  reconnect();
  client.publish("outTopic", "ESPClient_Connected");
  // ... and resubscribe
  client.subscribe(topicRele1,1);

}

void loop() {

  yield();
    
  // Verifica Conexão
  if (!client.connected()) {
    reconnect();
  }

  // Loop MQTT
  client.loop();
  
  // Verificação OTA
  OTAHandle();
  
  // Calculo das Medições
  MeterHandle();

  yield();
    
}

// Calculo das Medições
void MeterHandle(){

   switch (status){
      case(1):
      
          /* Calculo dos Tempos de Processamento */    
          ant = fim;
          initial = micros();
          Serial.print("Tempo em Espera [us]: ");
          espera = (initial - ant);
          Serial.println(espera);
          Serial.println("=======================");
          calcMeter();
          fim = micros();
          Serial.println("=======================");
          Serial.print("Tempo de Processamento [us]: ");
          proc = (fim - initial);
          Serial.println(proc);
          Serial.print("Tempo de Total do Loop [us]: ");
          Serial.println(espera+proc);
          
      break;
   }
}

// Calculo das Medições
void calcMeter(){

   /* Calculo e Envio das Medições */
  
  // Função EnergyMonitor 
  emon1.calcVI(20,500);
  // Print Medições    
  //emon1.serialprint();

  // Recebimento dos Paramentos
  Vrms = emon1.Vrms;
  Irms = emon1.Irms;
  realPower = emon1.realPower;
  powerFactor = emon1.powerFactor;

  // Limites Inferiores
  if (Vrms < 1) Vrms = 0;
  if (Irms < 0.1) Irms = 0;
  if (realPower < 1) realPower = 0;
  if (powerFactor < 0.01) powerFactor = 0;
  
  Serial.println("=======================");
  Serial.println("Tensao:");
  Serial.println(Vrms);
  Serial.println("Corrente:");  
  Serial.println(Irms);
  Serial.println("realPower:");
  Serial.println(realPower);
  Serial.println("powerFactor:");  
  Serial.println(powerFactor);
  Serial.println("=======================");

  // Conversão double to String
  SVrms = String(Vrms,2);
  SIrms = String(Irms,2);
  SrealPower = String(realPower,2);
  SpowerFactor = String(powerFactor,2);
   
  // Join to payload
  payload += SVrms;
  payload += "&";
  payload += SIrms;
  payload += "&";
  payload += SrealPower;
  payload += "&";
  payload += SpowerFactor;
   
  // Envio do Pacote MQTT
  sendPayload(payload, topicCurrent);
  payload = "";

  // Flag Stop Calculo
  status = 0;
    
}


// Configura o Tirmer 0
//Sub rotina ISR do Timer a cada 1 Segundo
void timerInterrupt(void *z){

  // Flat Start Calculo
   status++;
   
}

// Configura o Tirmer 0
void configTimer(){
  
  os_timer_setfn(&tmr0, timerInterrupt, NULL);      //Indica ao Timer qual sera sua Sub rotina.
  os_timer_arm(&tmr0, 1000, true);                  //Inidica ao Timer seu Tempo em mS e se sera repetido ou apenas uma vez (loop = true)
  
}

// Converte mac to string
String macToStr(const uint8_t* mac){
  
  String result;
  for (int i = 0; i < 6; ++i) {

      result += String(mac[i], 16);

      if (i < 5)
          result += ':';
  }
  return result;
}


// Configura a Conexão Wifi
void configWifi(const char* ssid, const char* password ){
  
  String clientName;
  WiFiClient wifiClient;

  //Comunicação Serial
  Serial.begin(115200);
  
  Serial.println();
  Serial.print("Connecting to ")  ;
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Aguardando Conexão
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  // Conectado
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Cliente
  clientName += "esp8266-";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += " - ";
  clientName += String(micros() & 0xff, 16);
  Serial.println(clientName);

}


// Manipula Mensagem Recebidas
void callback(char* topic, byte* payload, unsigned int length) {
  
  // Convert payload to String
  String PayloadString = String((char*)payload);
  // Convert String to CharArray
  char PayloadChar[5];
  // Convert String to CharArray  
  PayloadString.toCharArray(PayloadChar, 5);
  // Pring Payload
  Serial.println("=======================");
  Serial.println(PayloadChar);
  Serial.println("=======================");

  // Rele 1
  if (strcmp(topic,topicRele1) == 0) {
      
      if ( strcmp(PayloadChar,"desl") == 0){
         
        digitalWrite(RELE1, LOW);
         
      }else if ( strcmp(PayloadChar,"liga") == 0){
        
        digitalWrite(RELE1, HIGH);
        
      }
  }

  yield();
  
}


// Envia MQTT Payload
void sendPayload(String payload, char* topic) {
  
  if (!client.connected()) {
    if (client.connect((char*) clientName.c_str())) {
      Serial.println("Connected to MQTT broker again");
      Serial.print("Topic is: ");
      Serial.println(topic);
    }
    else {
      Serial.println("MQTT connect failed");
      Serial.println("Will reset and try again...");
      abort();
    }
  }

  if (client.connected()) {
    Serial.print("Sending payload: ");
    Serial.println(payload);

    if (client.publish(topic, (char*) payload.c_str())) {
      Serial.println("Publish ok");
    }
    else {
      Serial.println("Publish failed");
    }
  }

}


void reconnect() {
  
  // Loop until we're reconnected
  while (!client.connected()) {

    Serial.print("Attempting MQTT connection...");
    
    // Create a random client ID
    clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
    
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "ESPClient_Connected");
      // ... and resubscribe
      client.subscribe(topicRele1,1);
      //client.subscribe(topicRele2,1);
      
    } else {
      
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      
    }
  }
}


// Configura Atualização Over the Air
void configOTA(){

  Serial.println("Configuring OTA");
  
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("PowerBoard-1");

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"esp8266");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Configured");
  
}

// Verificação OTA
void OTAHandle(){
   ArduinoOTA.handle();

}





