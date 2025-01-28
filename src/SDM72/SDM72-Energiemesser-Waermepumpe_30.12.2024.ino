/*
 * Modbus - RS485 Kopplung mit Energiemesser SDM72 auf http://ip-adr und 
 * MQTT Publish unter info/SDM72 
*/
#define vers "Version: 30.12.2024"
#define WLAN_Name   "WLAN_ Name" //muss mit dem Namen des WLAN gefüllt werden!
#define PASSWORT    "Passwort_vom_WLAN" //muss mit dem Passwort des WLAN gefüllt werden
#define MQTT_SERVER ""    //Adresse des MQTT-Servers, z.B. #define MQTT_SERVER "192.168.178.42"
#define IE          ""                  //Volkszaehler UUID fuer  Inneneinheit, falls Volkszaehler genutzt wird
#define AE          ""                  //Volkszaehler UUID fuer Ausseneinheit, falls Volkszaehler genutzt wird
#define phase_L1    ""                  //Volkszaehler UUID fuer Phase die nicht Innen- und nicht Ausseneinheit ist, falls Volkszaehler genutzt wird
#define energie_e   ""                  //Volkszaehler UUID fuer Energie, falls Volkszaehler genutzt wird
 
#define  LED 2 //D4 GPIO 02
#define  AN LOW
#define  AUS HIGH
#define RS485_Richtung 12  //pIN FÜR DIE RS485 in DER Richtung umzuschalten //D6=GPIO12
const char* ssid = WLAN_Name;
const char* password = PASSWORT;
char uuid_inneneinheit[] = IE, //Volkszaehler UUID
     uuid_ausseneinheit[]= AE, //Volkszaehler UUID 
     uuid_L1[]           = phase_L1, //Volkszaehler UUID
     uuid_E[]            = energie_e; //Volkszaehler UUID
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h> // Server WiFiServer server(80),  Server WiFiClient client1??? 
#include <Arduino_JSON.h>
#include "ota.h" 
#include "Internetzeit.h"
#include "mqtt.h" 
#include "Telnet2Serial.h" //ermoeglicht die Ausgabe von Werten via Telnet auf Port 23
#include <WiFiClient.h> 
ESP8266WebServer server(80); // create the webserver on port 80
ESP8266WiFiMulti WiFiMulti;
//die folgenden 2 Zeilen für shelly1 Emulation
const char* settings = "{\"device\":{\"type\":\"SHPLG-S\",\"mac\":\"4022D8836C9E\",\"hostname\":\"shellyplug-s-4022D8836C9E\",\"num_outputs\":1,\"num_meters\":1},\"wifi_ap\":{\"enabled\":false,\"ssid\":\"shellyplug-s-4022D8836C9E\",\"key\":\"\"},\"wifi_sta\":{\"enabled\":true,\"ssid\":\"FRITZ!UWE\",\"ipv4_method\":\"dhcp\",\"ip\":null,\"gw\":null,\"mask\":null,\"dns\":null},\"wifi_sta1\":{\"enabled\":false,\"ssid\":null,\"ipv4_method\":\"dhcp\",\"ip\":null,\"gw\":null,\"mask\":null,\"dns\":null},\"ap_roaming\":{\"enabled\":false,\"threshold\":-70},\"mqtt\": {\"enable\":false,\"server\":\"192.168.33.3:1883\",\"user\":\"\",\"id\":\"shellyplug-s-4022D8836C9E\",\"reconnect_timeout_max\":60.000000,\"reconnect_timeout_min\":2.000000,\"clean_session\":true,\"keep_alive\":60,\"max_qos\":0,\"retain\":false,\"update_period\":30},\"coiot\": {\"enabled\":true,\"update_period\":15,\"peer\":\"\"},\"sntp\":{\"server\":\"time.google.com\",\"enabled\":true},\"login\":{\"enabled\":false,\"unprotected\":false,\"username\":\"admin\"},\"pin_code\":\"\",\"name\":\"Uwes Shelly PlugS\",\"fw\":\"20230913-113421/v1.14.0-gcb84623\",\"pon_wifi_reset\":false,\"discoverable\":false,\"build_info\":{\"build_id\":\"20230913-113421/v1.14.0-gcb84623\",\"build_timestamp\":\"2023-09-13T11:34:21Z\",\"build_version\":\"1.0\"},\"cloud\":{\"enabled\":true,\"connected\":true},\"timezone\":\"Europe/Berlin\",\"lat\":52.406200,\"lng\":9.793200,\"tzautodetect\":true,\"tz_utc_offset\":3600,\"tz_dst\":false,\"tz_dst_auto\":true,\"time\":\"16:37\",\"unixtime\":1699976268,\"led_status_disable\":false,\"debug_enable\":false,\"allow_cross_origin\":false,\"actions\":{\"active\":false,\"names\":[\"btn_on_url\",\"out_on_url\",\"out_off_url\"]},\"hwinfo\":{\"hw_revision\":\"dev-prototype\",\"batch_id\":0},\"max_power\":1800,\"led_power_disable\":false,\"relays\":[{\"name\":\"Was ist das ChanelName\",\"appliance_type\":\"Uwe General\",\"ison\":false,\"has_timer\":false,\"default_state\":\"off\",\"auto_on\":0.00,\"auto_off\":0.00,\"schedule\":false,\"schedule_rules\":[],\"max_power\":1800}],\"eco_mode_enabled\":true}";
const char* shelly = "{\"type\":\"SHPLG-S\",\"mac\":\"4022D8836C9E\",\"auth\":false,\"fw\":\"20230913-113421/v1.14.0-gcb84623\",\"discoverable\":false,\"longid\":1,\"num_outputs\":1,\"num_meters\":1}";
char buf[1700];
WiFiClient   wifiClient;  
ADC_MODE   (ADC_VCC);
//                                  E-L1       E-L2       E-L3   Eges-rel   Eges-abs
unsigned long wertE[7][5] = {   {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} }; // E1, E2, E3, Egesamt_relativ. Egesamt_absolut
unsigned long wertE_alt=0;
unsigned long wertL[] {0,0,0,0};//L1, L2, L3, Lgesamt
unsigned long anz=1, miniL[] = {10000,10000,10000,10000}, maxiL[] = {0,0,0,0}, mittwL[] = {0,0,0,0}; //Minimum, Maximum, Mittelwert von L1, L2,L3, Lgesamt jeweils ab Mitternacht
unsigned long timer_led=0, t_alt=0, t_aktuell, t_shelly=0;
int errorcnt=0, err_wait=0; //CRC-Fehler auf der RS485 Schnittstelle zaehlen
boolean valid;
unsigned long summe_fuer_mittelw[]={0,0,0,0};
boolean h_flag=false;
byte data[5][6] = { //1.Byte=Adresse, 2Byte=ModbusFunktion, 3-4Byte=Register, 5-6Byte=Anz. Daten der Antwort über RS485 zu senden
                //1.Byte, 2.Byte, 3.Byte, 4.Byte, 5.Byte, 6.Byte
                  { 0x01,   0x04,   0x00,   0x0C,   0x00,   0x02}, //Leistung Phase1 Modbus
                  { 0x01,   0x04,   0x00,   0x0E,   0x00,   0x02}, //Leistung Phase2 Modbus
                  { 0x01,   0x04,   0x00,   0x10,   0x00,   0x02}, //Leistung Phase3 Modbus
                  { 0x01,   0x04,   0x00,   0x34,   0x00,   0x02}, //Leistung gesamt Modbus
                  { 0x01,   0x04,   0x01,   0x8C,   0x00,   0x02}  //Energie  gesamt Modbus
                   };
long d[5];
//*********************************************************************
void setup() {
  //Serial.begin(115200);
  //Serial.print("\n SDM72 auslesen ");Serial.println(vers);
  pinMode(LED, OUTPUT);
  digitalWrite (LED,AUS);
  pinMode(RS485_Richtung, OUTPUT); 
  digitalWrite (RS485_Richtung,HIGH);
  Serial.begin(9600, SERIAL_8N1) ; //Geschw. 9600 bit/sec., 8 Bit/Zeichen, kein Paritätsbit, 1 Stopp-Bit
  Serial.swap(); //TX auf D8 (GPIO15)und RX auf D7 (GPIO13) umschalten
  //************************************************/Serial.print("\n SDM72 auslesen ");Serial.println(vers);
  wifistart();  //WLAN starten
  showTime(); //Internetzeit initialisieren
}
//*********************************************************************************
void wifistart() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname("SDM72-Energy"); 
  WiFi.begin(ssid, password); //Serial.println("\nConnecting to WLAN");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print(".");
  }
  sprintf(buf,"\nconnected, address=%s; Hostname=%s, Version= %s\r\n",WiFi.localIP().toString().c_str(),WiFi.hostname().c_str(),vers);
  //Serial.print(buf);

  server.on("/", handleRoot); // handleRoot() ist die Funktion die aufgerufen wird, wenn nur die IP ohne Zusatz eingegeben wird
  server.on("/status", []() { // bei IP/status wird die Funktion sdm72_status aufgerufen und damit shellyemulationsdaten ausgegeben    
    sdm72_status();
    });
  server.on("/settings", []() { // bei IP/settings wird die Funktion sdm72_settings aufgerufen und damit shellyemulationsdaten ausgegeben       
    sdm72_settings();
    });
  server.on("/shelly", []() { // bei IP/shelly wird die Funktion sdm72_shelly aufgerufen und damit shellyemulationsdaten ausgegeben       
    sdm72_shelly();
    });
  server.onNotFound(handleNotFound); //bei IP/irgendwas... wird die Funktion handleNotFound aufgerufen
  server.begin();
  sprintf(buf,"\nconnected, address=%s; Hostname=%s, Version= %s\r\n",WiFi.localIP().toString().c_str(),WiFi.hostname().c_str(),vers);
  //Serial.print(buf);
}
//*********************************************************************************
void loop() {
  uwes_ota();
  yield();
  server.handleClient(); //gibt es eine Anfrage an den Server?
  if (WiFi.status() != WL_CONNECTED) {delay(60000); wifistart();} //wenn WLAN nicht läuft, versuche WLAN neu zu starten
  t_aktuell=millis();
  if((t_aktuell-t_shelly)<8000 || (t_aktuell-t_shelly)>14000) {//shelly emulation wird alle 10 Sekunden abgefragt, wenn letzte Abfrage länger als 8 Sekunden her,  nicht ausführen
    if(t_aktuell - t_alt >10000) { // 1000 Millisekunden Pause
      t_alt = t_aktuell;
      yield();
      showTime(); //Internetzeit aktualisieren
      yield();
      if(err_wait == 0) {
        if( sdm72() >= 0)  {
          vz(); //alle Werte ohne Fehler gelesen, dann Werte zum Volkszaehler
          valid=true;
        }
        else {      
          errorcnt++;
          err_wait=5;
          valid=false;
        }
      }
      else err_wait--;
      yield();
    }
  }
  if(strlen(MQTT_SERVER) > 1) mqtt(wertL, wertE[0], buf); //aktuelle Leistung und Energie via MQTT veröffentlichen, Energiewerte werden um Mitternacht rückgesetzt
  if(t_aktuell - timer_led > 200) digitalWrite(LED,AUS);  //LED wird bei poisiver CRC Prüfung eingeschaltet und timer gesetzt, nach 200ms wieder aus
  freeheap(); // zum Testen messen wie klein der freeheap werden kann
}
//*********************************************************************************
int sdm72() { //Werte aus Stromzaehler SDM72 lesen
  //return(1); fuer Testzwecke, wenn kein Zaehler angeschlossen ist
  for( int n=0; n<=4;n++) { //alle Werte vom Zähler lesen, ein Wert falsch, alle Werte verwerfen
    if( (d[n]=rs485_send_and_rec(data[n]))> 999999990) return -1; 
    delay(10);
  }
  //sprintf(buf,"errorcnt=%d, L1=%d, L2=%d, L3=%d, Lges=%d, E=%d\n\r",errorcnt, d[0], d[1],d[2],d[3],d[4]); telnet_write(buf); //Werte über telnet ausgeben
  if(tm.tm_hour == 0 || anz==1) { //0 Uhr?, oder erster Durchlauf, Werte auf Null setzen und Energiwerte um einen Tag schiften
    if(h_flag == false) {
      h_flag =true;
      wertE_alt = d[4];
      anz=1;
      for(int n=5; n>=0; n--) { // älteste Wert in 6 fliegt raus, 5. Tag nach 6. Tag, 4. Tag nach 5 Tag usw bis 0. Tag nach 1.Tag
        for(int m=0; m<=4; m++) {
          wertE[n+1][m] = wertE[n][m];  
        }
      }
      errorcnt = 0;
      for(int n=0; n<=3;n++) { //Mitternacht, Werte ruecksetzen, 0=L1, 1=L2, 2=L3, 3=Lges
        miniL[n] = 10000000;
        maxiL[n] = 0;
        mittwL[n]= 0;  
        summe_fuer_mittelw[n] = 0; 
      }
    } 
  }
  if(tm.tm_hour != 0 ) h_flag = false;
  for(int n=0; n<=3;n++) { //Leistung lesen, Minimal, Maximal und Mittelwert berechnen
    wertL[n]= ((d[n]/100)+5)/10;
    if(wertL[n] < miniL[n]) miniL[n]=wertL[n];          //Minimalwert überprüfen und ggf. speichern
    if(wertL[n] > maxiL[n]) maxiL[n]=wertL[n];          //Maximalwert überprüfen und ggf. speichern
    summe_fuer_mittelw[n] = summe_fuer_mittelw[n] + wertL[n]; //Mittelwert berechnen, 1.Summe aller Werte bilden
    if(anz!=0) mittwL[n] = summe_fuer_mittelw[n] / anz;          //und dann durch die Anzahl teilen, sollte anz Null sein, Absturz!
  }
  anz++;
  for(int n=0;n<=2;n++) wertE[0][n] = (mittwL[n]*(anz/36)/10); //Leistung W in kWh umrechnen, da alle 10 Sekunden abgefragt wir geteilt durch 360              
  wertE[0][4]= d[4] ; //absolute Energie 
  wertE[0][3]= wertE[0][4] - wertE_alt; //absolute Energie minus Energiewert von Mitternacht entspricht relativen_Energiewert_des_Tages
  //////// TELNET ///////////  if(anz % 60 ==0) { //nach jedem x. Aufruf (entspricht y Sekunden) ausgeben
  //////// TELNET ///////////    sprintf(buf, "%02d:%02d:%02d L1:%4d, L2:%4d, L3:%4d, Lgesamt:%4d, E_ges_absolut:%5d, E_gesamt_relativ:%5d, wertE_alt: %d, anz:%4d, MittelW_L2 %4d, Errorcnt: %d\n\r", 
  //////// TELNET ///////////             tm.tm_hour,tm.tm_min,tm.tm_sec, wertL[0],wertL[1],wertL[2],wertL[3], wertE[0][4], wertE[0][3], wertE_alt, anz,mittwL[1], errorcnt ); 
  //////// TELNET ///////////    telnet_write(buf); //Serial.print(buf);
  //////// TELNET ///////////  } 
  return 0;
}
//*********************************************************************************
unsigned long rs485_send_and_rec(byte* zu_senden)  { 
  byte b;
  unsigned long nach_float_wandeln=0; //am Ende dieser Funktion beschrieben, wie das funktioniert
  unsigned int crc = 0xFFFF;                        //Wert ist bei RS485 Modbus zur CRC Prüfung definiert
  //##########RS485 SENDEN . . .
  int cnt=0;
  for (int n = 0; n < 6; n++) crc = crc16(crc, zu_senden[n]); //es ist über sechs Byte CRC zu bilden
  Serial.flush(); //Seriellen Sendepuffer leeren bzw. warten bis dieser geleert ist
  digitalWrite(RS485_Richtung, HIGH); //RS485 bzw.  MODBUS ist halbduplex, RS485 Chip in Senderichtung schalten
  while(Serial.available() > 0 ) Serial.read(); //Falls noch Daten (woher auch immer) im Lesepuffer sein sollten diese jetzt ohne Auswertung lesen == EMpfangspuffer leeren
  for( int n=0; n<6;n++) Serial.write( zu_senden[n]);  //es sind sechs Zeichen zu senden
  Serial.write( crc%256); //CRC Low-Byte  senden
  Serial.write( crc/256); //CRC High-Byte senden
  Serial.flush(); // Funktion wartet, dass Schreibpuffer leer
  //###########RS485 EMPFANGEN . . . 
  digitalWrite(RS485_Richtung, LOW);  //RS485 senden ist fertig, jetzt RS485 Chip auf empfangen umschalten die Antwort auswerten 
  crc = 0xFFFF; //crc neu initialisieren für die Antwortprüfung
  unsigned long ti_watchdog=millis();
  while(millis() < (ti_watchdog+700)) { //um die 9 erwarteten Zeichen zu empfangen sind max. 700 Millisekunden Zeit
    if( Serial.available() > 0) {
      b = Serial.read();
      buf[cnt]=b;  
      //////////////////sprintf(buf,"W: %2x ,cnt:%d\n\r", b, cnt);telnet_write(buf); //optional und zum Testen Telnet-Ausgabe
      if(cnt>=3 && cnt <=6) nach_float_wandeln = (nach_float_wandeln * 256) + b;
      if(cnt>=0 && cnt <=6) crc = crc16(crc, b);
      if(++cnt == 9) break; //es werden 9 zeichen (0-8) erwartet
    }
    yield();
  }
  if( cnt < 9 || buf[7] != crc%256 || buf[8] != crc/256 ) { //CRC in Ordnung?
    //sprintf(buf,"errorcnt=%d, CRC-Fehler berechnet %04x, empfangen %04x, cnt=%02d, ", errorcnt, crc, buf[8]*256+buf[7],cnt); telnet_write(buf); //optional und zum Testen Telnet-Ausgabe
    return 999999999;
  }
  timer_led = millis();
  digitalWrite (LED,AN); //blaue Board LED an und in Loop-Schleife nach 100ms wieder aus
  float f = *(float*)&nach_float_wandeln; //siehe Kommentar am Ende der Funktion
  //sprintf(buf,"f=%10.8f", f);
  if(f<0.0 ) f=0.0; //manchmal ist f -1.8xxx groß, Fehler vom Zaehler, da negativ hier nicht möglich ist expiliet auf Null setzen falls negativ if(f<0.0 && f>-5.0) f=0.0;
  unsigned long ul =  long (f*1000); // kaufmaennisch runden
  //sprintf(buf+strlen(buf)," ,f=%10.8f,  ul=%08d\n\r", f, ul); telnet_write(buf);  //telnet_write("hier Telnet\n\r");
  return ul; //if(ul<12000000) return ul;  else return 999999998; //Plausibilitätsprüfung
  // UMRECHNUNG von float nach unsigned long
  //Internet: https://forum.arduino.cc/t/4-byte-hex-mit-dem-uno-nach-ieee-754-float-umrechen/215800
  //Du musst dazu aber den Umweg über einen Pointer nehmen. Einfach die Adresse eines unsigned long nehmen, auf einen float* casten und dereferenzieren
  //unsigned long i = 0x47265931;float f = *(float*)&i;
  //& ist die Adresse des Integers. Diese wird auf einen Zeiger auf float gecastet. Und mit * hat man wieder den Wert.
}
//***************************************************************************************************************
unsigned int crc16(unsigned int crc, byte b) { //Blockprüfzeichen BCC (2 Byte, 16 Bit) berechnen
  //vor dem 1. Aufruf ist crc = 0xFFFF, Wert ist bei RS485 Modbus zur CRC Prüfung definiert
  crc ^= b;
  for(int n=0;n< 8;n++) {
    if((crc&1) ==1) crc=(crc >> 1) ^ 0xA001;  //Wert 0xA001 ist bei RS485 Modbus zur CRC Prüfung definiert
    else crc = crc >> 1;                      // >>1 bedeutet Shift 1 Bit nach rechts, ^ bitweise Exclusiv Oder
  } 
  return crc;
}
//*********************************************************************
unsigned long freeheap() {
  static unsigned long minfreeheap=100000;
  if(minfreeheap > ESP.getFreeHeap()) minfreeheap = ESP.getFreeHeap();
  return minfreeheap;
}
//*********************************************************************
void handleRoot() { // bei Aufruf der IP-Adresse ohne Ergaenzung, z.B. 192.168.178.28
if((millis()-t_shelly) > 8000&& (millis()-t_shelly)<14000) return;
unsigned long energie_vortag1;
  showTime(); //Internetzeit aktualisieren
              sprintf(buf,"     S D M 7 2  -  E N E R G I E  /  L E I S T U N G\n");
  sprintf(buf+strlen(buf),"==========================================================\n\n");
  for(int n=6;n>=0;n--) {
    sprintf(buf+strlen(buf),"Tag: %3d,   Energie[Wh] E-L1 :%6d,  E-L2:%6d,  E-L3:%6d, E-ges-24h:%5d, E-ges-abs: %6d\n",n*(-1), wertE[n][0], wertE[n][1], wertE[n][2], wertE[n][3], wertE[n][4]) ;                                
  } 
  sprintf(buf+strlen(buf),"\nH E U T E :\n-----------\n");                                
  sprintf(buf+strlen(buf),"           Leistung [W] L-L1:  %5d,  L-L2: %5d,  L-L3: %5d, L-ges: %5d, \n", wertL[0], wertL[1], wertL[2], wertL[3] );                                
  sprintf(buf+strlen(buf),"  minimale Leistung [W] L-L1:  %5d,  L-L2: %5d,  L-L3: %5d, L-ges: %5d, \n", miniL[0], miniL[1], miniL[2], miniL[3] );                                
  sprintf(buf+strlen(buf),"  maximale Leistung [W] L-L1:  %5d,  L-L2: %5d,  L-L3: %5d, L-ges: %5d, \n", maxiL[0], maxiL[1], maxiL[2], maxiL[3] );                                
  sprintf(buf+strlen(buf),"MittelWert Leistung [W] L-L1:  %5d,  L-L2: %5d,  L-L3: %5d, L-ges: %5d, \n", mittwL[0], mittwL[1], mittwL[2], mittwL[3] );                                
  sprintf(buf+strlen(buf),"\n\n\nErrorcnt: %d, %s,laeuft seit %d Tagen, Start am %s, \naktuelle Zeit: %02d:%02d:%02d, RSSI: %d, Spannung [mV]: %d, ESP.getFreeHeap(): %d, min heap: %d", 
                 errorcnt, vers, (millis()/3600000)/24, start_datum_zeit, tm.tm_hour,tm.tm_min,tm.tm_sec, wifi_station_get_rssi(),ESP.getVcc(), ESP.getFreeHeap(), freeheap()); 
  server.send(200, "text/plain", buf);
}
//*********************************************************************
void sdm72_status() { //wird von Venus/Victron/Cerbo-GX/Raspberry alle 10 Sekunden aufgerufen Venus glaubt es mit einem Shelly zu tun zu haben
  //unsigned long e=wertE[0][3]*60, p=wertL[3]; //shelly speichert die Energie pro Minute, daher *60
  if(valid==true) {
    sprintf(buf,"{\"meters\":[{\"power\":%d.0,\"overpower\":0.00,\"is_valid\":true,\"timestamp\":3,\"counters\":[0.000, 0.000, 0.000],\"total\":%d}]}",
                wertL[3], wertE[0][3]*60); //shelly speichert die Energie pro Minute, daher *60
    server.send(200, "text/plain", buf);
  }
  t_shelly=millis(); // in der Zeit von 8 Sekunden nach dem Aufruf wird auf die naechste Statusabfrage gewartet, hier wird der Timer t_shelly gesetzt
}
//*********************************************************************
void sdm72_settings() { // für Initialisierung des Perl-Programms in Venus wichtig . . . 
  //char buf[1500];sprintf(buf,"IP/settings\n"); Serial.print(buf);
  server.send(200, "text/html", settings);
}
//*********************************************************************
void sdm72_shelly() { // für Initialisierung des Perl-Programms in Venus wichtig . . . 
  server.send(200, "text/html", shelly);
}
//*********************************************************************
void handleNotFound() {
  sprintf(buf,"File Not Found\n\nURI: ,%d\nMethod:%d\nArguments:%d\n, server.argName: \%d",server.uri(),server.method(),server.args()/*,server.argName*/ );
  server.send(404, "text/plain", buf);  
}
//*********************************************************************************
void vz() { //nur bei Änderung der Werte oder nach x Sekunden in den VZ schreiben
  static unsigned int cnt=0, A=0, I=0, L1=0,E=0;
  if(cnt % 5 ==0) {
    if( ((I*21)/20) < wertL[1] || ((I*20)/21) > wertL[1] || (cnt%1800)==0 ) { //nur bei 5% Änderung oder jedes 1800. Mal (=30 Min.) den VZ aufrufen
      I=wertL[1];
      volkszaehler(uuid_inneneinheit, I);  // Leistung in W
    }
    if( ((A*21)/20) < wertL[2] || ((A*20)/21) > wertL[2] || ((cnt+450)%1800)==0 ) { //nur bei 5% Änderung oder jedes 600. Mal den VZ aufrufen
      A=wertL[2];
      volkszaehler(uuid_ausseneinheit, A);  // Leistung in W
    }
    if( ((L1*21)/20) < wertL[0] || ((L1*20)/21) > wertL[0] || ((cnt+900)%1800)==0 ) { //nur bei 5% Änderung oder jedes 600. Mal den VZ aufrufen
      L1=wertL[0];
      volkszaehler(uuid_L1, L1);  // Leistung in W
    }
    if( ((E*21)/20) < wertE[0][4] || ((E*20)/21) > wertE[0][4] || ((cnt+1350)%1800)==0 ) { //nur bei 5% Änderung oder jedes 600. Mal den VZ aufrufen
      E=wertE[0][4];
      volkszaehler(uuid_E, E);  
    }
  }
  cnt++;
}
//***************************************************************************
void volkszaehler(char * uuid, unsigned int power) { // 
  yield();
  if(uuid[0]>'\0') {
    sprintf(buf,"http://demo.volkszaehler.org/middleware.php/data/%s.json?operation=add&value=%d",uuid, power);
    yield();
    HTTPClient http;
    http.begin(wifiClient, buf); //HTTP Beispiel:  demo.volkszaehler.org/middleware.php/data/xxxxxxxx-e7f7-11e7-9c5f-bd0d27701812.json?operation=add&value=22.90
    int httpCode = http.GET();
  }
  yield();
}
//*************************************************************************************

