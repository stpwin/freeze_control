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
#define LED_SYNC_SUCCESS_PIN LED_BUILTIN//15
#define TIMER_MODE_PIN 14
#define MAX_SCHEDULES 10
#define MAX_RELAY 2
#define SERVER_PORT 8090
#define UDP_PORT 8888
#define EEPROM_SYSTEM_CONFIG_SIZE 4
#define EEPROM_RELAY_OUTPUT_CONFIG_SIZE EEPROM_SYSTEM_CONFIG_SIZE + (MAX_RELAY * 4)

uint8_t relays_output_pin[MAX_RELAY] = {12, 13};
uint8_t relays_manual_button_pin[MAX_RELAY] = {4, 5};
// uint8_t relays_mode[MAX_RELAY] = {0}; // for separate mode per relay


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
auto bangkokTz = TimeZone::forZoneInfo(&zonedb::kZoneAsia_Bangkok, &bangkokProcessor);
acetime_t nowSeconds;
static NtpClock ntpClock;
static SystemClockLoop systemClock(nullptr /*reference*/, nullptr /*backup*/);

uint8_t time_sync_done = 0;
uint8_t schedule_changed = 0;
uint8_t relay_mode_changed = 0;
uint8_t relays_output_changed = 0;
uint8_t eeprom_save_left_secs = 10; // 10 seconds
uint8_t mode = 0; // 0 = Timer, 1 = Manual
// volatile uint8_t relays_manual_button_pressed[MAX_RELAY] = {1}; 
uint8_t relays_output[MAX_RELAY] = {0};

uint8_t print_current_time = 0;
uint8_t reply_schedule = 0;
uint8_t ntp_first_run = 1;
unsigned long ntp_millis = 0;
unsigned long second_millis = 0;
unsigned long debounce_millis = 0;

volatile uint8_t button0 = 0;
volatile uint8_t button1 = 0;
volatile uint8_t button2 = 0;

ICACHE_RAM_ATTR void ButtonTimerInterrupt(){
  button0 = 1;
}

ICACHE_RAM_ATTR void ButtonManualRelayInterrupt0(){
  button1 = 1;
}

ICACHE_RAM_ATTR void ButtonManualRelayInterrupt1(){
  button2 = 1;
}

void buttonLoop(){
  if (button0){
    button0 = 0;

    mode = 0;
    relay_mode_changed = 1;
    resetEEPROMSaveCounter();
  } else if (button1){
    button1 = 0;

    mode = 1;
    relays_output[0] = !relays_output[0];

    relays_output_changed = 1;
    ProcessRelaysOutput();
    resetEEPROMSaveCounter();
  } else if (button2){
    button2 = 0;

    mode = 1;
    relays_output[1] = !relays_output[1];
    relays_output_changed = 1;
    ProcessRelaysOutput();
    resetEEPROMSaveCounter();
  }
}

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
    Serial.println(wifi.get("ssid"));
    delay(200);
  });
}

void init_pin(){
  pinMode(LED_SYNC_SUCCESS_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TIMER_MODE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TIMER_MODE_PIN), ButtonTimerInterrupt, FALLING);

  for (uint8_t relay_index = 0; relay_index < MAX_RELAY; relay_index++){
    pinMode(relays_output_pin[relay_index], OUTPUT);
    pinMode(relays_manual_button_pin[relay_index], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(relays_manual_button_pin[0]), ButtonManualRelayInterrupt0, FALLING);
  attachInterrupt(digitalPinToInterrupt(relays_manual_button_pin[1]), ButtonManualRelayInterrupt1, FALLING);
}

void init_communication(){
  msgOut.Magic = MESSAGE_MAGIC;
}

void SaveSystemConfig(){
  Serial.println("Writing system's config from EEPROM");
  EEPROM.put(0, mode);
  if (EEPROM.commit())
  {
    Serial.println("EEPROM successfully committed");
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }
}

void LoadSystemConfig(){
  Serial.println("Loading system's config from EEPROM");
  EEPROM.get(0, mode);
  Serial.print("Mode: ");
  Serial.println(mode);
}

void SaveRelayOutputConfig(){
  Serial.println("Writing relay output started to EEPROM");
  for (uint8_t i = 0; i < MAX_RELAY; i++){
    int address = (i * 4) + EEPROM_SYSTEM_CONFIG_SIZE;
    EEPROM.put(address, relays_output[i]);
  }

  if (EEPROM.commit())
  {
    Serial.println("EEPROM successfully committed");
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }
}

void LoadRelayOutputConfig(){
  Serial.println("Loading relay output started to EEPROM");
  for (uint8_t i = 0; i < MAX_RELAY; i++){
    int address = (i * 4) + EEPROM_SYSTEM_CONFIG_SIZE;
    EEPROM.get(address, relays_output[i]);
  }
}

void SaveScheduleConfig(){
  Serial.println("Writing schedule to EEPROM");
  size_t struct_size = sizeof(Schedule_t);
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++){
    int address = (i * struct_size) + EEPROM_RELAY_OUTPUT_CONFIG_SIZE;
    EEPROM.put(address, schedules[i]);
  }
  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }
}

void LoadScheduleConfig(){
  Serial.println("Loading schedule from EEPROM");
  size_t struct_size = sizeof(Schedule_t);
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++){
    int address = (i * struct_size) + EEPROM_RELAY_OUTPUT_CONFIG_SIZE;
    EEPROM.get(address, schedules[i]);
  }
}

void ProcessScheduler(uint8_t hour, uint8_t minute){
  if (mode == 1){
    return;
  }
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++){
    if (schedules[i].Enable){
      // uint8_t sOnHour = (uint8_t)(schedules[i].TimeOn / 100);
      // uint8_t sOnMin = (uint8_t)(schedules[i].TimeOn % 100);
      uint16_t curTime = (hour * 100) + minute;
      // Between On time and Off time
      if (curTime >= schedules[i].TimeOn && curTime < schedules[i].TimeOff){
        if (!relays_output[schedules[i].RelayId]){
          relays_output[schedules[i].RelayId] = 1;
          relays_output_changed = 1;
          resetEEPROMSaveCounter();
          ProcessRelaysOutput();
          Serial.print("Relay ");Serial.print(schedules[i].RelayId);Serial.println("-> ON");
        }

      //Current time > Off time

        /*
      ****จะแก้ time overlap ยังไงจ้ะ****
      CASE: Schedule มีมากกว่า 1 Timeline ที่ใช้ Relay ร่วมกัน
            SCHEDULE A       •→――――――――――――→• 
                             ↑ เปิด(ON_time) ↑ ปิด แต่โดนอันล่างทับ ดังนั้นห้ามปิด(Overlap) << ตรงนี้แหละมีปัญหา
            SCHEDULE B       •→――――――――――――――――――――――→•
                             ↑ เปิด(ON_time)           ↑ ปิดจริงๆ(No overlap)
            SCHEDULE C •→――――――――――――――――――――――→•
                       ↑ เปิด(ON_time)           ↑ ปิด ห้ามปิด โดนอันบนทับ(Overlap)
            SCHEDULE D                                     •→――――――――→•
                                                           ↑ เปิด      ↑ ปิด(No overlap)
      SOLUTION: Loop through schedules where there are use same relay and store it to an array
                Find overlap time by ON_time A (INSIDE) Andljdhfgajdhfgakdhfga kjhfga jlkdhfgaldkfjhaldfkja hlfkj hvlkjdhfvlkjhsdflvkjhsdlfkhg
                Select one SCHEDULE to be use WHERE OFF_time > others schedule
      */
      } else if (curTime >= schedules[i].TimeOff) { 
        if (relays_output[schedules[i].RelayId]){
          relays_output[schedules[i].RelayId] = 0;
          relays_output_changed = 1;
          resetEEPROMSaveCounter();
          Serial.print("Relay ");Serial.print(schedules[i].RelayId);Serial.println("-> OFF");
          ProcessRelaysOutput();
        }
      }
    }
  }
}

void tick()
{
  // Serial.println(nowSeconds);

  if ((uint8_t)digitalRead(LED_SYNC_SUCCESS_PIN) != time_sync_done)
  {
    digitalWrite(LED_SYNC_SUCCESS_PIN, time_sync_done);
  }

  if (time_sync_done) {
    acetime_t now = systemClock.getNow();
    auto bangkokTime = ZonedDateTime::forEpochSeconds(now, bangkokTz);
    ProcessScheduler(bangkokTime.hour(), bangkokTime.minute());
    if (!print_current_time){
      print_current_time = 1;
      bangkokTime.printTo(SERIAL_PORT_MONITOR);
      SERIAL_PORT_MONITOR.println("");
    }
  }

  checkEEPROMConfigChanged();
}

void checkEEPROMConfigChanged(){
  if (schedule_changed || relay_mode_changed || relays_output_changed){
    if (eeprom_save_left_secs > 0){
      eeprom_save_left_secs--;
    } else {
      eeprom_save_left_secs = 10;

      if (schedule_changed){
        schedule_changed = 0;
        SaveScheduleConfig();
      }

      if (relay_mode_changed){
        relay_mode_changed = 0;
        SaveSystemConfig();
      }

      if (relays_output_changed){
        relays_output_changed = 0;
        SaveRelayOutputConfig();
      }
    }
  }
}

void resetEEPROMSaveCounter(){
  eeprom_save_left_secs = 10;
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
    // case Command_t::SetRelayMode: // for separate relay
    //   Serial.println("SetRelayMode");
    //   msgOut.Command = Command_t::SetRelayMode;
    //   if (ValidRequestRelay()){
    //     relays_mode[msgIn.Parameter0] = msgIn.Parameter1;
    //     msgOut.Parameter0 = msgIn.Parameter0;
    //     msgOut.Parameter1 = msgIn.Parameter1;
    //   }
    //   break;
    case Command_t::SetRelayMode:
      Serial.println("SetRelayMode");
      msgOut.Command = Command_t::SetRelayMode;

      mode = msgIn.Parameter0;
      relay_mode_changed = 1;
      resetEEPROMSaveCounter();
      msgOut.Parameter0 = msgIn.Parameter0;
      msgOut.Parameter1 = msgIn.Parameter1;
      break;
    case Command_t::ManualSetRelay:
      Serial.println("ManualSetRelay");

      Serial.print("Parameter0: ");Serial.println(msgIn.Parameter0);
      Serial.print("Parameter1: ");Serial.println(msgIn.Parameter1);
      Serial.print("Parameter2: ");Serial.println(msgIn.Parameter2);

      msgOut.Command = Command_t::ManualSetRelay;
      if (ValidRequestRelay()){
        mode = 1;
        relays_output[msgIn.Parameter0] = msgIn.Parameter1;
        msgOut.Parameter0 = msgIn.Parameter0;
        msgOut.Parameter1 = msgIn.Parameter1;
        relays_output_changed = 1;
        resetEEPROMSaveCounter();
        ProcessRelaysOutput();
      }
      break;
    // case Command_t::GetRelayMode: // for separate mode per relay
    //   Serial.println("GetRelayMode");
    //   msgOut.Command = Command_t::GetRelayMode;
    //   if (ValidRequestRelay()){
    //     msgOut.Parameter0 = msgIn.Parameter0; //relay Index
    //     msgOut.Parameter1 = relays_mode[msgIn.Parameter0]; //relay status
    //   }
    //   break;
    case Command_t::GetRelayMode:
      Serial.println("GetRelayMode");
      msgOut.Command = Command_t::GetRelayMode;
      if (ValidRequestRelay()){
        msgOut.Parameter0 = mode; //relay Index
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
        schedule_changed = 1;
        resetEEPROMSaveCounter();
        // eeprom_save_left_secs = 10;
      }
      break;
    case Command_t::GetSchedule:
      Serial.println("GetSchedule");
      msgOut.Command = Command_t::GetSchedule;
      // if ()
      if (ValidScheduleId(msgIn.Parameter0)){
        reply_schedule = 1;
        msgOut.Parameter0 = msgIn.Parameter0; //Schedule Index
        scheduleOut = schedules[msgIn.Parameter0];
        // Serial.print("TimeOn: ");
        // Serial.println(scheduleIn.TimeOn);
        // Serial.print("TimeOff: ");
        // Serial.println(scheduleIn.TimeOff);
        // Serial.print("Enable: ");
        // Serial.println(scheduleIn.Enable);
        // Serial.print("RelayId: ");
        // Serial.println(scheduleIn.RelayId);
        // schedules[msgIn.Parameter0] = scheduleIn;
        // schedule_changed = 1;
        // eeprom_save_left_secs = 10;
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
        // client.write((uint8_t *)&msgOut, sizeof(msgOut));
        if (reply_schedule){
          reply_schedule = 0;
          uint8_t send[sizeof(msgOut) + sizeof(scheduleOut)];
          memcpy(send, &msgOut, sizeof(msgOut));
          memcpy(send+sizeof(msgOut), &scheduleOut, sizeof(scheduleOut));
          client.write((uint8_t *)&send, sizeof(send));
        } else {
          client.write((uint8_t *)&msgOut, sizeof(msgOut));
        }
        
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

void syncTimeNow(){
  Serial.println("Getting internet time..");
  nowSeconds = ntpClock.getNow();
  time_sync_done = (uint8_t)(nowSeconds > 646832014);
  if (time_sync_done){
    Serial.println("Synced.");
    auto bangkokTime = ZonedDateTime::forEpochSeconds(nowSeconds, bangkokTz);
    systemClock.setNow(bangkokTime.toEpochSeconds());
  } else {
    Serial.println("Failed.");
  }
}

void setup()
{
  digitalWrite(LED_BUILTIN, LOW);
  EEPROM.begin(60);

  LoadSystemConfig();
  LoadRelayOutputConfig();
  LoadScheduleConfig();

  ProcessRelaysOutput();

  systemClock.setup();
  init_pin();
	init_hardware();
  init_wifi();
  init_communication();

  wifi.connect();
  udp.begin(UDP_PORT);
  server.begin();

  
}

void loop()
{
	wifi.loop();
  systemClock.loop();

  if (millis() - debounce_millis >= 300){
    debounce_millis = millis();
    buttonLoop();
  }

  if (wifi.connected()){
    tcpLoop();

    if (!ntp_first_run && ntpClock.isSetup() && !time_sync_done){

      if (millis() - second_millis >= 10000){
        syncTimeNow();
        second_millis = millis();
      }
    }

    if ((millis() - ntp_millis) >= 3600000 || ntp_first_run){ //Sync time every 1 hour
      ntp_millis = millis();
      
      
      if (ntp_first_run){
        Serial.println("Setting up ntp clock...");
        ntpClock.setup();
        
        if (!ntpClock.isSetup()) {
          Serial.println(F("Something went wrong."));
          return;
        }

        second_tick.attach(1, tick);
        ntp_first_run = 0;
      }

      syncTimeNow();
    }
  }
}
