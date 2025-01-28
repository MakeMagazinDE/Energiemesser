#include <PubSubClient.h>         //MQTT Lib
int mqtt_port = 1883; 
WiFiClient espClient;
PubSubClient mqtt_client (espClient);
//****************************************************************************
void mqtt_callback(char* topic, byte* payload, unsigned int laenge) { //Subscibe (Empfange) MQTT Nachricht, wird nicht verwendet
  Serial.printf("Message arrived [%s]: ",topic);
  for (int i = 0; i < laenge; i++) Serial.print((char)payload[i]); Serial.println();
}
//****************************************************************************
boolean reconnect() {  
  static String mac = WiFi.macAddress();
  char buf[20];
  for(int n=0; n<18;n++) buf[n]= mac[n]; buf[19]='\0';
  if(mqtt_client.connect(buf)) { //beim MQTT Server anmelden, Name ist egal, könnte auch z.B."Fidel" heißen
      Serial.println("connected"); // Once connected, publish an announcement...
      mqtt_client.publish("info/sdm72", "hier MQTT SDM72 ENERGIEMESSER");
      mqtt_client.subscribe("cmnd/sdm72"); // ... and resubscribe
      Serial.printf("mit eindeutigem Bezeichner, hier Mac-Adresse < %s > \"anmelden\"\n",buf);
  }
  return mqtt_client.connected();
}
//****************************************************************************
void client_connected_unblocking() {
  if(!mqtt_client.connected()) {
    unsigned long  ti_last_connection=0;
    if(millis()-ti_last_connection > 5000) {
      ti_last_connection = millis();
      if(reconnect()) ti_last_connection=0;
    }
  }
  else mqtt_client.loop();
}
//****************************************************************************
void mqtt(unsigned long *pwertL, unsigned long *pwertE, char *pbuf) {
  static boolean erster_aufruf = false;
  static unsigned long ti_pause = millis();
  if(erster_aufruf == false) {
    erster_aufruf = true;
    mqtt_client.setServer(MQTT_SERVER, mqtt_port);
    mqtt_client.setCallback(mqtt_callback); //MQTT Eingangs-/Empfangsroutine festlegen
    Serial.printf("hier erster Aufruf und Initialisierung, MQTT_SERVER: %s, mqtt_port: %d\n", MQTT_SERVER, mqtt_port);
  }
  mqtt_client.loop(); //damit das Empfangen/Subscribe funktioniert
  if (!mqtt_client.connected()) client_connected_unblocking();
  if(millis()-ti_pause> 30000) { //nur 2x pro Minute ausführen
    ti_pause = millis();
    sprintf(pbuf,"{\"zeit\":\"%02d%02d%02d\",\"leistung\":{\"L1\":%d,\"L2\":%d,\"L3\":%d,\"Lgesamt\":%d},\"energie\":{\"E1_24h\":%d,\"E2_24h\":%d,\"E3_24h\":%d,\"Eges_24h\":%d,\"Egesamt\":%d}}",
    tm.tm_hour,tm.tm_min,tm.tm_sec, pwertL[0],pwertL[1],pwertL[2],pwertL[3]   ,pwertE[0],  pwertE[1], pwertE[2], pwertE[3], pwertE[4]);
    mqtt_client.publish("info/sdm72",pbuf); //PUBLIZIEREN
    Serial.printf("hier mqtt\n");
  }
}
