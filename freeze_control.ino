#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiConnector.h>
#include <time.h>
#include <Ticker.h>
#include <AceTime.h>
#include <EEPROM.h>

using namespace ace_time;
using namespace ace_time::clock;

enum Command_t : uint8_t{
    InvalidMessage = 0,
    InvalidRequest,
    InvalidParameter,
    Ping,
    Pong,
    SyncTime,
    SetRelayMode,
    ManualSetRelay,
    GetRelayMode,
    GetRelayState,
    SetSchedule,
    GetSchedule
};

struct __attribute((__packed__)) Message_t{
    uint16_t Magic; //H - 2 Bytes
    Command_t Command; //B - 1 Bytes
    uint16_t Parameter0; //H - 2 Bytes
    uint16_t Parameter1; //H - 2 Bytes
    uint16_t Parameter2; //H - 2 Bytes
    uint8_t raw[4];
} msgIn, msgOut;

typedef struct __attribute((__packed__)) Schedule_t{
  uint16_t TimeOn; //
  uint16_t TimeOff;
  uint8_t Enable;
  uint8_t RelayId;
};

String ssid = "IoT";
String pass = "59122420124";

#define MESSAGE_MAGIC 0xBEEF
#define LED_SYNC_SUCCESS 14
#define MAX_SCHEDULES 10
#define MAX_RELAY 2
#define SERVER_PORT 8090
#define UDP_PORT 8888
// #define NTP_PACKET_SIZE 48

uint8_t relays_output[MAX_RELAY] = {0};
uint8_t relays_output_pin[MAX_RELAY] = {12, 13};
uint8_t relays_mode[MAX_RELAY] = {0};

Schedule_t schedules[MAX_SCHEDULES];
Schedule_t scheduleIn = {.TimeOn = 0, .TimeOff = 0, .Enable = 0, .RelayId = 0};
Schedule_t scheduleOut = {.TimeOn = 0, .TimeOff = 0, .Enable = 0, .RelayId = 0};

const char* time_server = "time1.navy.mi.th";

     
IPAddress local_ip = {10, 0, 0, 3};
IPAddress gateway = {10, 0, 0, 1};
IPAddress subnet = {255, 255, 255, 240};
IPAddress dns = {1, 1, 1, 1};

WiFiUDP udp;
WiFiConnector wifi(ssid, pass);
WiFiServer server(SERVER_PORT);
Ticker second_tick;

static BasicZoneProcessor bangkokProcessor;
auto bangkokTz = TimeZone::forZoneInfo(
      &zonedb::kZoneAsia_Bangkok, &bangkokProcessor);
acetime_t nowSeconds;
static NtpClock ntpClock;
static SystemClockLoop systemClock(nullptr /*reference*/, nullptr /*backup*/);

// uint8_t packetBuffer[NTP_PACKET_SIZE];
uint8_t time_sync_done = 0;
uint8_t schedule_has_change = 0;
uint16_t schedule_save_left = 10; // 10 seconds

void init_hardware(){
  Serial.begin(115200);
  WiFi.disconnect(true);
  delay(1000);
  Serial.flush();
  Serial.println();
  Serial.println();
  Serial.println("will be started in 500ms..");
}

void init_wifi(){
  WiFi.config(local_ip, gateway, subnet, dns);
  wifi.init();
  wifi.on_connected([&](const void* message)
  {
    Serial.print("WIFI CONNECTED WITH IP: ");
    Serial.println(WiFi.localIP());
  });

  wifi.on_connecting([&](const void* message)
  {
    Serial.print("Connecting to ");
    Serial.println(wifi.get("ssid") + ", " + wifi.get("password"));
    delay(200);
  });
}

void init_pin(){
  pinMode(LED_SYNC_SUCCESS, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t relay_index = 0; relay_index < MAX_RELAY; relay_index++){
    pinMode(relays_output_pin[relay_index], OUTPUT);
  }
}

void init_communication(){
  msgOut.Magic = MESSAGE_MAGIC;
}

void SaveSchedule(){
  Serial.println("Writing schedule to EEPROM");
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++){
    int address = i * sizeof(Schedule_t);
    EEPROM.put(address, schedules[i]);
  }
  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }
}

void LoadSchedule(){
  Serial.println("Loading schedule from EEPROM");
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++){
    int address = i * sizeof(Schedule_t);
    EEPROM.get(address, schedules[i]);
  }
}

void ProcessScheduler(uint8_t hour, uint8_t minute){
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++){
    if (schedules[i].Enable){
      // uint8_t sOnHour = (uint8_t)(schedules[i].TimeOn / 100);
      // uint8_t sOnMin = (uint8_t)(schedules[i].TimeOn % 100);
      uint16_t curTime = (hour * 100) + minute;

      // Between On time and Off time
      if (curTime >= schedules[i].TimeOn && curTime < schedules[i].TimeOff){

        if (!relays_output[schedules[i].RelayId]){
          relays_output[schedules[i].RelayId] = 1;
          Serial.print("Relay ");Serial.print(schedules[i].RelayId);Serial.println("-> ON");
          ProcessRelaysOutput();
        }

      //Current time > Off time
      } else if (curTime >= schedules[i].TimeOff) {
        if (relays_output[schedules[i].RelayId]){
          relays_output[schedules[i].RelayId] = 0;
          Serial.print("Relay ");Serial.print(schedules[i].RelayId);Serial.println("-> OFF");
          ProcessRelaysOutput();
        }
        
        //Turn off
      }
    }
  }
}

uint8_t printCurTime = 0;
void tick()
{
    acetime_t now = systemClock.getNow();
    time_sync_done = (now > 646832014);
    digitalWrite(LED_SYNC_SUCCESS, time_sync_done);

    if (time_sync_done) {
      auto bangkokTime = ZonedDateTime::forEpochSeconds(now, bangkokTz);
      ProcessScheduler(bangkokTime.hour(), bangkokTime.minute());
      if (!printCurTime){
        printCurTime = 1;
        bangkokTime.printTo(SERIAL_PORT_MONITOR);
      }
      // SERIAL_PORT_MONITOR.println("");
    }

    if (schedule_has_change){
      if (schedule_save_left > 0){
        schedule_save_left--;
      } else {
        schedule_has_change = 0;
        schedule_save_left = 10;
        // Serial.println();
        SaveSchedule();
      }

    }
}

void setup()
{
  EEPROM.begin(60);
  LoadSchedule();

  systemClock.setup();
  init_pin();
	init_hardware();
  init_wifi();
  init_communication();

  

  wifi.connect();
  udp.begin(UDP_PORT);
  server.begin();
}

void ProcessRelaysOutput(){
  Serial.println("ProcessRelaysOutput");
  for (uint8_t relay_index = 0; relay_index < MAX_RELAY; relay_index++){
    digitalWrite(relays_output_pin[relay_index], relays_output[relay_index]);
  }
}

uint8_t ValidRelayId(uint8_t relayId){
   return relayId < MAX_RELAY;
}

uint8_t ValidRelayMode(uint8_t mode){
   return mode < 2;
}

uint8_t ValidRequestRelay(){
  // msgIn.Parameter0; //Relay Index
  // msgIn.Parameter1; //Relay Mode
  if (!ValidRelayId(msgIn.Parameter0)){
    msgOut.Command = Command_t::InvalidParameter;
    msgOut.Parameter0 = Command_t::ManualSetRelay;
    msgOut.Parameter1 = 1;
    return 0;
  }
  if (!ValidRelayMode(msgIn.Parameter1)){
    msgOut.Command = Command_t::InvalidParameter;
    msgOut.Parameter0 = Command_t::ManualSetRelay;
    msgOut.Parameter1 = 2;
    return 0;
  }
  return 1;
}

uint8_t ValidScheduleId(uint8_t scheduleId){
  return scheduleId < MAX_SCHEDULES;
}

uint8_t ValidRequestSchedule(){
  // msgIn.Parameter0; //Schedule Index
  // msgIn.Parameter1; //Relay Mode
  if (!ValidScheduleId(msgIn.Parameter0)){
    msgOut.Command = Command_t::InvalidParameter;
    msgOut.Parameter0 = Command_t::SetSchedule;
    msgOut.Parameter1 = 1;
    return 0;
  }
  if (!ValidRelayId(scheduleIn.RelayId)){
    msgOut.Command = Command_t::InvalidParameter;
    msgOut.Parameter0 = Command_t::SetSchedule;
    msgOut.Parameter1 = 2;
    return 0;
  }
  // if (msgIn.Parameter1 > 1){
  //   msgOut.Command = Command_t::InvalidParameter;
  //   msgOut.Parameter0 = Command_t::ManualSetRelay;
  //   msgOut.Parameter1 = 2;
  //   return false;
  // }
  return 1;
}

void ProcessRequest(){
  switch (msgIn.Command)
    {
    case Command_t::Ping:
      msgOut.Command = Command_t::Pong;
      Serial.println("Ping");
      break;
    case Command_t::SetRelayMode:
      Serial.println("SetRelayMode");
      msgOut.Command = Command_t::SetRelayMode;
      if (ValidRequestRelay()){
        relays_mode[msgIn.Parameter0] = msgIn.Parameter1;
        msgOut.Parameter0 = msgIn.Parameter0;
        msgOut.Parameter1 = msgIn.Parameter1;
      }
      break;
    case Command_t::ManualSetRelay:
      Serial.println("ManualSetRelay");

      Serial.print("Parameter0: ");Serial.println(msgIn.Parameter0);
      Serial.print("Parameter1: ");Serial.println(msgIn.Parameter1);
      Serial.print("Parameter2: ");Serial.println(msgIn.Parameter2);

      msgOut.Command = Command_t::ManualSetRelay;
      if (ValidRequestRelay()){
        relays_output[msgIn.Parameter0] = msgIn.Parameter1;
        msgOut.Parameter0 = msgIn.Parameter0;
        msgOut.Parameter1 = msgIn.Parameter1;
        ProcessRelaysOutput();
      }
      break;
    case Command_t::GetRelayMode:
      Serial.println("GetRelayMode");
      msgOut.Command = Command_t::GetRelayMode;
      if (ValidRequestRelay()){
        msgOut.Parameter0 = msgIn.Parameter0; //relay Index
        msgOut.Parameter1 = relays_mode[msgIn.Parameter0]; //relay status
      }
      break;
    case Command_t::GetRelayState:
      Serial.println("GetRelayState");
      msgOut.Command = Command_t::GetRelayState;
      if (ValidRequestRelay()){
        msgOut.Parameter0 = msgIn.Parameter0; //relay Index
        msgOut.Parameter1 = relays_output[msgIn.Parameter0]; //relay status
      }
      break;
    case Command_t::SetSchedule:
      Serial.println("SetSchedule");
      msgOut.Command = Command_t::SetSchedule;

      
      if (ValidRequestSchedule()){
        msgOut.Parameter0 = msgIn.Parameter0; //Schedule Index

        Serial.print("TimeOn: ");
        Serial.println(scheduleIn.TimeOn);
        Serial.print("TimeOff: ");
        Serial.println(scheduleIn.TimeOff);
        Serial.print("Enable: ");
        Serial.println(scheduleIn.Enable);
        Serial.print("RelayId: ");
        Serial.println(scheduleIn.RelayId);
        schedules[msgIn.Parameter0] = scheduleIn;
        schedule_has_change = 1;
        schedule_save_left = 10;
      }
      break;
    default:
      msgOut.Command = Command_t::InvalidRequest;
      break;
    }
}

void clearOutData(){
  msgOut.Command = Command_t::InvalidMessage;
  msgOut.Parameter0 = 0;
  msgOut.Parameter1 = 0;
  msgOut.Parameter2 = 0;
  msgOut.raw[0] = 0;
  msgOut.raw[1] = 0;
  msgOut.raw[2] = 0;
  msgOut.raw[3] = 0;
}

void tcpLoop(){
  WiFiClient client = server.available();
  if (!client) return;

  Serial.println("Client available.");
  while (client.connected())
  {
    int available = client.available();
    if (available){
      Serial.print("Size: ");
      Serial.println(available);
    }
    

    if (available > 2)
    {
      client.read((uint8_t *)&msgIn, sizeof(msgIn));
      int available1 = client.available();
      if (available1){
        Serial.print("Next available size: ");
        Serial.println(available1);
        client.read((uint8_t *)&scheduleIn, sizeof(scheduleIn));
        available1 = client.available();
        Serial.print("Still available size?: ");
        Serial.println(available1);
        Serial.println("Clear available buffer.");
        while(client.available()){ //Clear buffer
          client.read();
        }
      }
      if (msgIn.Magic == MESSAGE_MAGIC)
      {
        Serial.println("Magic match.");
        clearOutData();
        ProcessRequest();
        client.write((uint8_t *)&msgOut, sizeof(msgOut));
      }
      else
      {
        Serial.println("Magic failed!");
        client.write((uint8_t *)&msgOut, sizeof(msgOut));
        break;
      }
    } else {
      // break;
    }
  }
  client.stop();
  
  Serial.println("Client disonnected.");
}

unsigned long ntp_millis = 0;
unsigned long second_millis = 0;
uint8_t ntp_first_run = 1;
void loop()
{
	wifi.loop();
  systemClock.loop();

  if (wifi.connected()){
    tcpLoop();

    if ((millis() - ntp_millis) >= 3600000 || ntp_first_run){ //Sync time every 1 hour
      ntp_millis = millis();
      if (ntp_first_run){
        Serial.println("Setting up ntp clock...");
        ntpClock.setup();
        
        if (!ntpClock.isSetup()) {
          Serial.println(F("Something went wrong."));
          return;
        }
        Serial.println("Done.");
        nowSeconds = ntpClock.getNow();
        auto bangkokTime = ZonedDateTime::forEpochSeconds(nowSeconds, bangkokTz);
        systemClock.setNow(bangkokTime.toEpochSeconds());
        second_tick.attach(1, tick);
        ntp_first_run = 0;
        return;
      }

      nowSeconds = ntpClock.getNow();
      auto bangkokTime = ZonedDateTime::forEpochSeconds(nowSeconds, bangkokTz);
      systemClock.setNow(bangkokTime.toEpochSeconds());

    }
  }
}
