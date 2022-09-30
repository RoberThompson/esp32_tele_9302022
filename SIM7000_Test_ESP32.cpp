//Authored by: Daniel J Manla

#include <Wire.h>                             //I2C
#include <SPIFFS.h>                           //ESP32 Flash Storage Filesystem  
#include <Update.h>                           //ESP32 Firmware Updating
#include <ArduinoJson.h>                      //JSON handling and parsing, for Telemetry Data
#include <WiFi.h>                             //WiFi Soft-AP 
#include "ESPAsyncWebServer.h"                //Web server for semi-remote updates
#include <DFRobot_SIM7000.h>                  //Interfacing with SIM7000 Cellular Modem
#include <StreamUtils.h>                      //Used to optimize file access
#include "fileupload_page.h"                  //Webpage HTML for update page
#include <ArduinoNmeaParser.h>                //Parses sentences received from modem GNSS module
#include "SparkFun_AVR_ISP_Programming.h"     //Used for programming Mega, using ESP32  
#include "SdFat.h"                            //SdFat by Bill Greiman
#include <EEPROM.h>
//---------------------------------------------------------------//
//------------------------Pin Definitions------------------------//
//---------------------------------------------------------------//
#define I2C_ADR 0x03
#define I2C_SCL 1
#define I2C_SDA 2
#define I2C_FRQ 400000
#define MEGA_OE   9
#define MEGA_RST 10
#define SPI_MOSI 11
#define SPI_SCLK 12
#define SPI_MISO 13
#define MODEM_RI 39
#define MODEM_TX 40
#define MODEM_RX 41
#define MODEM_PWR 4
#define MODEM_RST 6
#define MODEM_DTR 38
#define EEPROM_SIZE 4
//---------------------------------------------------------------//
//---------------------------------------------------------------//
#define THINGSBOARD_CLID "X000_0000"
#define FIRMWARE_VERSION "09.06.2022"
#define THINGSBOARD_PASS  "test1234"
//---------------------------------------------------------------//
//----------------------GPS NMEA Handling------------------------//
//---------------------------------------------------------------//
void onRmcUpdate(nmea::RmcData const);
void onGgaUpdate(nmea::GgaData const);
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);
//---------------------------------------------------------------//
//------------------Cellular NTP Sync Settings-------------------//
//---------------------------------------------------------------//
#define NTP_SYNC_INTERVAL 120000
#define NTP_URL "0.pool.ntp.org"
uint32_t ntpSyncEndTime = 0x00;
tm CurrentTime;
//---------------------------------------------------------------//
//-----------------Automatic Reset Variables---------------------//
//---------------------------------------------------------------//
unsigned long modemResetTimer = 0;
const int rstTime = 14400000;
//---------------------------------------------------------------//
//-----------------Web Server Initialization---------------------//
//---------------------------------------------------------------//
AsyncWebServer server(80);
const char *soft_ap_ssid = THINGSBOARD_CLID;
const char *soft_ap_password = THINGSBOARD_PASS;
//---------------------------------------------------------------//
//---------------------------------------------------------------//
TaskHandle_t WebTaskHandle;
TaskHandle_t I2CTaskHandle;
void I2CTask( void * parameter);
void WebTask( void * parameter);

HardwareSerial modemSerial(2);
DFRobot_SIM7000 sim7000(&modemSerial);
SFE_AVR_ISP myISP(SPI_MISO, SPI_MOSI, SPI_SCLK, MEGA_RST); 

#define KEYTABLE_SIZE 200
int   keyCount = 0;
bool  DSVAL_Change = false;
bool  DSVAL_Update[KEYTABLE_SIZE] = {0};
union floatToBytes  {
  uint8_t  asBytes[4];
  float asFloat;
} cmdValue, rxVal;
uint8_t systemState = 0xFF;

bool spiffsMounted = false;
bool sdCardMounted = false;
//---------------------------------------------------------------//
//----------------------------SD Card----------------------------//
//---------------------------------------------------------------//
#define SDCARD_SS_PIN 8
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#define error(msg) (Serial.println(F("error " msg)), false)
SdFat32 sd;
File32 sdFile;
//---------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//
#define POLL_TIMEOUT 5000
bool subscribed = false;
uint8_t failCount = 0;
uint32_t i2cPollStopTime = 0x00;
volatile int i2cReceive = 0x00;
volatile bool i2cRequest = false;
volatile uint8_t cmdByte = 0x00;
char keyBuffer[8] = {'\0'};
char valBuffer[8] = {'\0'};
char receiveBuffer[128] = { 0 };
volatile bool modem_DataReceived = false;
void IRAM_ATTR ModemRX() {
  modem_DataReceived = true;
}
//---------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------//

StaticJsonDocument<2500> keyStorage;
StaticJsonDocument<500> bufferedKeyStorage;
StaticJsonDocument<500> configStorage;

//-------------------------------------------------------//
//---------------------I2C Handlers----------------------//
//-------------------------------------------------------//
void IRAM_ATTR i2cReceiveHandler(int numBytes) {
  i2cReceive = numBytes;
  cmdByte = Wire.read();
  if (cmdByte == 0xFE) {
    for (int i = 0; i < 9; ++i) {
      if (i < 5) {
        keyBuffer[i] = Wire.read();
      } else if (i < 9) {
        cmdValue.asBytes[i - 5] = Wire.read();
      }
    }
  } else if (cmdByte == 0xFF) {
    for (int i = 0; i < 5; ++i) {
      keyBuffer[i] = Wire.read();
    }
    cmdValue.asFloat = keyStorage[keyBuffer].as<float>();
  }
}

void IRAM_ATTR i2cRequestHandler() {
  Wire.write(cmdByte);
  Wire.write((uint8_t*)keyBuffer, 5);
  Wire.write(cmdValue.asBytes, 4);
}

//-------------------------------------------------------//
//-------------------------------------------------------//
//-------------------------------------------------------//
bool checkValidKey(char* keyBuffer, int keySize);
void notFound(AsyncWebServerRequest *request);
bool checkUserWebAuth(AsyncWebServerRequest * request);
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
String humanReadableSize(const size_t bytes);
String processor(const String& var);
void configureWebServer(AsyncWebServer* webServer);
String listFiles(bool ishtml = false);

void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  *date = FS_DATE(CurrentTime.tm_year, CurrentTime.tm_mon, CurrentTime.tm_mday);
  *time = FS_TIME(CurrentTime.tm_hour, CurrentTime.tm_min, CurrentTime.tm_sec);
  *ms10 = CurrentTime.tm_sec & 1 ? 100 : 0;
}
void setup() {
  EEPROM.begin(EEPROM_SIZE);
  //---------------------------------------------------------------//
  //----------------------Pin Configuration------------------------//
  //---------------------------------------------------------------//
  pinMode(MODEM_PWR, INPUT);  //Modem Power Button
  pinMode(MODEM_RST, INPUT);  //Modem Reset
  pinMode(MODEM_RI, INPUT);
  pinMode(MODEM_TX, INPUT);
  pinMode(MODEM_RX, INPUT);
  pinMode(MODEM_DTR, INPUT);
  delay(1000);
  pinMode(MODEM_PWR, OUTPUT);  //Modem Power Button
  pinMode(MODEM_RST, OUTPUT);  //Modem Reset

  pinMode(MEGA_OE, OUTPUT);     //Output Enable
  digitalWrite(MEGA_OE, LOW);

  pinMode(MEGA_RST, OUTPUT);
  digitalWrite(MEGA_RST, HIGH);
  //---------------------------------------------------------------//
  //------------------------Serial Setup---------------------------//
  //---------------------------------------------------------------//
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  modemSerial.begin(3686400, SERIAL_8N1, 40, 41);
  //---------------------------------------------------------------//
  //---------------------------I2C Setup---------------------------//
  //---------------------------------------------------------------//
  while (!Wire.begin(I2C_ADR, I2C_SDA, I2C_SCL, I2C_FRQ)) {
    Serial.println("I2C Begin Failed");
    vTaskDelay(500);
  }
  Serial.println("I2C Ready");

  Wire.onReceive(i2cReceiveHandler);        //Parses packets that are received
  Wire.onRequest(i2cRequestHandler);        //Sends reply
  attachInterrupt(MODEM_RI, ModemRX, RISING);
  //---------------------------------------------------------------//
  //---------------------------ISP Setup---------------------------//
  //---------------------------------------------------------------//
  myISP.enableDebugging();
  myISP.begin();
  myISP.startProgramming();   // Go into programming mode
  myISP.getSignature();       // Attempt to read the signature from the ISP device
  myISP.stopProgramming();    // Exit programming mode

  Serial.print(F("Chip signature is: 0x"));
  if (myISP.currentSignature.sig[0] < 0x10) Serial.print(F("0"));
  Serial.print(myISP.currentSignature.sig[0], HEX);
  Serial.print(F(" 0x"));
  if (myISP.currentSignature.sig[1] < 0x10) Serial.print(F("0"));
  Serial.print(myISP.currentSignature.sig[1], HEX);
  Serial.print(F(" 0x"));
  if (myISP.currentSignature.sig[2] < 0x10) Serial.print(F("0"));
  Serial.println(myISP.currentSignature.sig[2], HEX);

  Serial.print(F("The chip type is: "));
  Serial.print(myISP.currentSignature.desc);
  //---------------------------------------------------------------//
  //----------------------SPI Flash Storage------------------------//
  //---------------------------------------------------------------//
  Serial.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(true)) {
    Serial.println("ERR: Cannot mount SPIFFS");
  }else{
    spiffsMounted = true;
    if(!SPIFFS.exists("/TELVALS.json")){
      Serial.println("json file created");
      File file = SPIFFS.open("/TELVALS.json", FILE_WRITE);
      file.close();
    }else{
      File file = SPIFFS.open("/TELVALS.json", FILE_READ);
      ReadBufferingStream bufferingStream(file, 128);
      deserializeJson(keyStorage, bufferingStream);
      serializeJson(keyStorage, Serial);
      file.close();
    }
  }
  //---------------------------------------------------------------//
  //-----------------------SD Card Storage-------------------------//
  //---------------------------------------------------------------//
  Serial.println("Mounting SD...");
  FsDateTime::setCallback(dateTime);
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("ERR: Cannot mount SD Card");
    configStorage["TOPIC"] = "XXXXXXX";
    configStorage["SUBSC"] = "XXXXXXX";
    configStorage["HOST"] = "XXXXXXX";
    configStorage["USER"] = "X000_0000";
    configStorage["CLID"] = "X000_0000";
    configStorage["PASS"] = "test1234";
    configStorage["APN"] = "hologram";
  }else{
    sdCardMounted = true;
    if(sd.exists("CONFIG.json")){
      sdFile.open("CONFIG.json", O_READ);
      deserializeJson(configStorage, sdFile);
      serializeJson(configStorage, Serial);
      Serial.println();
      sdFile.close();
    }else{
      configStorage["TOPIC"] = "XXXXXXX";
      configStorage["SUBSC"] = "XXXXXXX";
      configStorage["HOST"] = "XXXXXXX";
      configStorage["USER"] = "X000_0000";
      configStorage["CLID"] = "X000_0000";
      configStorage["PASS"] = "test1234";
      configStorage["APN"] = "hologram";
    }

    if (!sd.exists("TELVALS.json") && !spiffsMounted){
      Serial.println("creating json");
      if (!sdFile.open("TELVALS.json", O_RDWR | O_CREAT)) {
        error("json file failed");
      }
      sdFile.close();
    }else{
      sdFile.open("TELVALS.json", O_READ);
      deserializeJson(keyStorage, sdFile);
      serializeJson(keyStorage, Serial);
      Serial.println();
      sdFile.close();
    }
  }  
  //---------------------------------------------------------------//
  //----------------------WiFi Configuration-----------------------//
  //---------------------------------------------------------------//
  WiFi.mode(WIFI_AP);
  WiFi.softAP(configStorage["CLID"].as<char*>(), soft_ap_password);
  Serial.println("Server Ready");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  configureWebServer(&server);
  server.begin();
  //---------------------------------------------------------------//
  //---------------------------------------------------------------//
  xTaskCreatePinnedToCore(
    I2CTask,          //Task function
    "I2C",            //Name of the task
    5000,             //Task stack size
    NULL,             //Allows a configurable parameter for task
    1,                //Task is highest priority on core 1
    &I2CTaskHandle,   //Task Handle
    1);               //I2C uses Core 1

  int numOfResets = EEPROM.read(10);
  numOfResets++;
  EEPROM.write(10,numOfResets);
  EEPROM.commit();
  Serial.println(numOfResets);
  while (true) {
    switch (systemState) {
      modemResetTimer = millis();
      case 0:               //Initialize Modem
        //-----------------------------------------------------------//
        //----------------------Set Baud Rate------------------------//
        //-----------------------------------------------------------//
        Serial.println(configStorage["CLID"].as<char*>());
        while (!sim7000.setBaudRate(3686400)) {
          Serial.println("Baud change err");
        }
        modemSerial.begin(3686400, SERIAL_8N1, 40, 41);
        //-----------------------------------------------------------//
        //----------------------Disable Echo-------------------------//
        //-----------------------------------------------------------//
        while (!sim7000.toggleEcho(0)) {
          Serial.println("Echo set err");
        }
        //-----------------------------------------------------------//
        //----------------------Config RI Pin------------------------//
        //-----------------------------------------------------------//
        while (!sim7000.setRiPin(1)) {
          Serial.println("RI Pin set err");
        }
        //-----------------------------------------------------------//
        //------------------Verify IMEI is present-------------------//
        //-----------------------------------------------------------//
        while (!sim7000.getIMEI(receiveBuffer)) {
          Serial.println("IMEI Err");
        }
        Serial.println(receiveBuffer);
        //-----------------------------------------------------------//
        //----------------Verify SIM Card is present-----------------//
        //-----------------------------------------------------------//
        while (!sim7000.checkSIMStatus()) {
          Serial.println("SIM Err");
        }
        //-----------------------------------------------------------//
        //---------------Enable LTE RF Functionality-----------------//
        //-----------------------------------------------------------//
        while (!sim7000.setFunctionality(1)) {
          Serial.println("Full func Err");
        }
        //-----------------------------------------------------------//
        //-----------------Disable Non-LTE Reception-----------------//
        //-----------------------------------------------------------//
        while (!sim7000.setOnlyLTE(2)) {
          Serial.println("Only LTE Err");
        }
        //-----------------------------------------------------------//
        //----------Ensure LTE is favored for reception--------------//
        //-----------------------------------------------------------//
        while (!sim7000.setPreferredLTEMode(3)) {
          Serial.println("Prefer LTE Err");
        }
        //-----------------------------------------------------------//
        //-------------------Enable GNSS Module----------------------//
        //-----------------------------------------------------------//
         while (!sim7000.GNSS_Toggle(true)) {
          Serial.println("GNSS Err");
        }
        //-----------------------------------------------------------//
        //-----------------------------------------------------------//
        systemState = 1;
        break;
      case 1:         //Initialize IP and Service Provider
        keyStorage["REGRX"] = systemState;
        sim7000.getIpState(receiveBuffer);
        if (strstr(receiveBuffer, "INITIAL")) {
          sim7000.setServiceProvider(configStorage["APN"].as<char*>());
          systemState = 1;
        } else if (strstr(receiveBuffer, "START")) {
          systemState = 2;
        } else if (strstr(receiveBuffer, "GPRSACT")) {
          systemState = 4;
        } else {
          sim7000.closeNetwork();
          systemState = 1;
        }
        break;
      case 2:           //Enable GPRS Service
        keyStorage["REGRX"] = systemState;
        if (sim7000.enableService(20000)) {
          systemState = 3;
        } else {
          Serial.println("GPRS Err");
          systemState = 1;
        }
        break;
      case 3:           //Verify GPRS Service is Active
        keyStorage["REGRX"] = systemState;
        if (sim7000.waitForIpState("GPRSACT", 10000)) {
          systemState = 4;
        } else {
          Serial.println("GPRS Err");
          systemState = 1;
        }
        break;
      case 4:         //Verify IP Address has been acquired
        keyStorage["REGRX"] = systemState;
        memset(receiveBuffer, 0x00, 64);
        if (sim7000.getIpAddress(receiveBuffer)) {
          failCount = 0;
          systemState = 5;
          Serial.println(receiveBuffer);
        } else {
          Serial.println("IP Err");
          failCount++;
          if (failCount > 3) {
            systemState = 0xFF;
          }
        }
        break;
      case 5:         //Verify registration with cellular carrier
        keyStorage["REGRX"] = systemState;
        sim7000.checkNetStatus(receiveBuffer);
        if (strstr(receiveBuffer, "0,5") != NULL) {
          systemState = 6;
        } else if (strstr(receiveBuffer, "0,2") != NULL) {
          Serial.println("Cell Searching");
          systemState = 5;
        } else if (strstr(receiveBuffer, "0,3") != NULL) {
          Serial.println("Cell Reject");
          systemState = 0xFF;
        }
        break;
      case 6:       //Enable MQTT Service (Web App in official documentation)
        keyStorage["REGRX"] = systemState;
        if (sim7000.ToggleWebApp(2, configStorage["APN"].as<char*>())) {
          failCount = 0;
          systemState = 7;
        } else {
          Serial.println(F("Webapp Err"));
          failCount++;
          if (failCount > 3) {
            systemState = 0xFF;
          }
        }
        break;
      case 7:     //Configure MQTT Service
        keyStorage["REGRX"] = systemState;
        if (!sim7000.MQTT_Config("URL", configStorage["HOST"].as<char*>())) {
          Serial.println(F("Config URL Err"));
          systemState = 7;
          failCount++;
        } else if (!sim7000.MQTT_Config("USERNAME", configStorage["USER"].as<char*>())) {
          Serial.println(F("Config User Err"));
          systemState = 7;
          failCount++;
        } else if (!sim7000.MQTT_Config("PASSWORD", THINGSBOARD_PASS)) {
          Serial.println(F("Config Pass Err"));
          systemState = 7;
          failCount++;
        } else if (!sim7000.MQTT_Config("CLIENTID", configStorage["CLID"].as<char*>())) {
          Serial.println(F("Config CLID Err"));
          systemState = 7;
          failCount++;
        } else {
          Serial.println(F("MQTT Configured"));
          systemState = 8;
          failCount = 0;
        }

        if (failCount > 5) {
          systemState = 0xFF;
        }
        sim7000.bearerProfile_Configure(1, "APN", configStorage["APN"].as<char*>());
        sim7000.bearerProfile_Open(1);
        if(sim7000.NTP_Configure(NTP_URL, 12, 1, 2)){
            Serial.println("NTP Configured");
        }
        break;   
      case 8:     //Connect to MQTT Server/Broker (Thingsboard)
        if (sim7000.MQTT_ConnectStatus(10000)) {
          Serial.println(F("MQTT RDY"));
          systemState = 9;
          failCount = 0;
        } else if (sim7000.MQTT_Connect(20000)) {
          subscribed = false;
          keyStorage["REGRX"] = 128;
          Serial.println(F("MQTT CON"));
          if (sim7000.MQTT_Unsubscribe(configStorage["SUBSC"].as<char*>())) {
            Serial.println("UNSUB");
          }
          while (!subscribed) {
            Serial.println("NOT SUBBED");
            subscribed = sim7000.MQTT_Subscribe(configStorage["SUBSC"].as<char*>(), 1);
          }
          i2cPollStopTime = millis() + POLL_TIMEOUT;
          systemState = 9;
          failCount = 0;
        } else {
          subscribed = false;
          Serial.println(F("MQTT FAIL"));
          sim7000.MQTT_Disconnect();
          systemState = 8;
          failCount++;
          if (failCount > 1) {
            Serial.println(F("MQTT RST"));
            failCount = 0;
            systemState = 0xFF;
          }
        }

        break;
      case 9:
        if (modem_DataReceived) {
          modem_DataReceived = false;
          Serial.println("Case 9");
          modemSerial.setTimeout(50);
          if (modemSerial.find("SMSUB")) {
            Serial.println("SUB MSG");
            if (modemSerial.readBytesUntil(',', receiveBuffer, 128)) {
              modemSerial.read();
              StaticJsonDocument<100> subscriptionJson;
              DeserializationError err = deserializeJson(subscriptionJson, modemSerial);
              if(err.code() == DeserializationError::Ok){
                failCount = 0;
                serializeJson(subscriptionJson, Serial);
                JsonObject documentRoot = subscriptionJson.as<JsonObject>();
                for (JsonPair keyValue : documentRoot) {
                  char trimmedKey[6] = {'\0'};
                  for (int i = 0; i < 5; ++i) {
                    trimmedKey[i] = keyValue.key().c_str()[i];
                  }
                  keyStorage[trimmedKey] = subscriptionJson[keyValue.key()].as<float>();
                  bufferedKeyStorage[trimmedKey] = subscriptionJson[keyValue.key()].as<float>(); 
                }
                
              }else if(err.code() == DeserializationError::InvalidInput){
                Serial.print(F("Invalid input!"));
                failCount++;
              }
            }  
          }
        }

        if(failCount > 5){
          systemState = 0xFF;
        }else if(millis() > i2cPollStopTime){
          systemState = 10;
        }
        break;
      case 10:
        {
          if(millis() > ntpSyncEndTime){                     
            ntpSyncEndTime = millis() + NTP_SYNC_INTERVAL;
            sim7000.getTime(&CurrentTime);
            if(sim7000.GNSS_AT_Set(true, 1)) {
              modemSerial.find('$');
              while(modemSerial.available()){
                  parser.encode((char)modemSerial.read());
                  vTaskDelay(1);
              }
              Serial.printf("%d/%d/%d|%d:%d:%d\n", CurrentTime.tm_year, CurrentTime.tm_mon, CurrentTime.tm_mday,CurrentTime.tm_hour, CurrentTime.tm_min, CurrentTime.tm_sec);
            }   
          }

          JsonObject temp = bufferedKeyStorage.as<JsonObject>();
          if (temp && (temp.begin() != temp.end()) && !temp.isNull()) {
            Serial.println("SMPUB...");
            char cmdStr[256];
            int messageLength = 0;
            messageLength = measureJson(temp) + 1;
            Serial.println(numOfResets);  
            snprintf(cmdStr, 250, "AT+SMPUB=\"%s\",%d,1,0\r\n", configStorage["TOPIC"].as<char*>(), messageLength);
            if (sim7000.checkSendCmd(cmdStr, ">", 1000)) {
              serializeJson(temp, modemSerial);
              modemSerial.write("\r\n\032");
              serializeJson(temp, Serial);
              Serial.println();
              bufferedKeyStorage.clear();

              if(spiffsMounted){
                File file = SPIFFS.open("/TELVALS.json", FILE_WRITE);
                WriteBufferingStream bufferedFile(file, 128);
                serializeJson(keyStorage, file);
                file.close();
              }else if(sdCardMounted){
                sdFile.open("TELVALS.json", O_WRITE);
                serializeJson(keyStorage, sdFile);
                sdFile.close();
              }
              
              i2cPollStopTime = millis() + POLL_TIMEOUT;
              systemState = 9;
              break;
            } else {
              systemState = 8;
              break;
            }
          }
          if(millis() - modemResetTimer >= rstTime){
            systemState - 0xFF;
            break;
          }
          else{
            systemState = 9;
            break;
          }
        }
      case 0xFF:
        systemState = 0;
        keyStorage["REGRX"] = systemState;
        sim7000.restart(MODEM_PWR, MODEM_RST);
        vTaskDelay(10000);
        Serial.println(F("MODEM RST"));
        break;
    }
  }
}
//-----------------------------------------------------------//
//-----------------------------------------------------------//
//-----------------------------------------------------------//
void loop() {
}

void I2CTask( void * parameter) {
  while (true) {
    if (i2cReceive) {
      if (keyBuffer[0] != '\0') {
        if (!keyStorage.containsKey(keyBuffer) ) {
          if(checkValidKey(keyBuffer,5)){
            Serial.print("NEW KEY: ");
            Serial.println(keyBuffer);
            keyStorage[keyBuffer] = cmdValue.asFloat;
            bufferedKeyStorage[keyBuffer] = cmdValue.asFloat;
          }     
        } else if ( keyStorage[keyBuffer].as<float>() != cmdValue.asFloat) {
          bufferedKeyStorage[keyBuffer] = cmdValue.asFloat;
          keyStorage[keyBuffer] = cmdValue.asFloat;
        }
      }
      i2cReceive = false;
    }
    vTaskDelay(1);
  }
}
//----------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------//
bool checkValidKey(char* keyBuffer, int keySize){
  for(int i = 0; i < keySize; ++i){
    if(!((keyBuffer[i] >= 'a' && keyBuffer[i]  <= 'z') || (keyBuffer[i]  >= 'A' && keyBuffer[i]  <= 'Z')) ){
      return false;
    }
  }
  return true;
}


// list all of the files, if ishtml=true, return html rather than simple text
String listFiles(bool ishtml) {
  String returnText = "";
  Serial.println("Listing files stored on SPIFFS");
  File root = SPIFFS.open("/");
  File foundfile = root.openNextFile();
  if (ishtml) {
    returnText += "<table><tr><th align='left'>Name</th><th align='left'>Size</th><th></th><th></th></tr>";
  }
  while (foundfile) {
    if (ishtml) {
      returnText += "<tr align='left'><td>" + String(foundfile.name()) + "</td><td>" + humanReadableSize(foundfile.size()) + "</td>";
      returnText += "<td><button onclick=\"downloadButton(\'" + String(foundfile.name()) + "\', \'download\')\">Download</button>";
      returnText += "<td><button onclick=\"deleteButton(\'" + String(foundfile.name()) + "\', \'delete\')\">Delete</button>";
      if(strstr(foundfile.name(), ".bin")){
        returnText += "<td><button onclick=\"updateEspButton(\'" + String(foundfile.name()) + "\', \'updateEsp\')\">Update ESP32</button>";
        returnText += "<td><button onclick=\"updateMegaButton(\'" + String(foundfile.name()) + "\', \'updateMega\')\">Update Mega</button>";
      }
      returnText += "</tr>";
    } else {
      returnText += "File: " + String(foundfile.name()) + " Size: " + humanReadableSize(foundfile.size()) + "\n";
    }
    foundfile = root.openNextFile();
  }
  if (ishtml) {
    returnText += "</table>";
  }
  root.close();
  foundfile.close();
  return returnText;
}

String humanReadableSize(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}

String processor(const String& var) {
  if (var == "FIRMWARE") {
    return FIRMWARE_VERSION;
  }

  if (var == "FREESPIFFS") {
    return humanReadableSize((SPIFFS.totalBytes() - SPIFFS.usedBytes()));
  }

  if (var == "USEDSPIFFS") {
    return humanReadableSize(SPIFFS.usedBytes());
  }

  if (var == "TOTALSPIFFS") {
    return humanReadableSize(SPIFFS.totalBytes());
  }
}

void configureWebServer(AsyncWebServer* webServer) {
  // configure web server

  // if url isn't found
  webServer->onNotFound(notFound);

  // run handleUpload function when any file is uploaded
  webServer->onFileUpload(handleUpload);

  // visiting this page will cause you to be logged out
  webServer->on("/logout", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->requestAuthentication();
    request->send(401);
  });

  // presents a "you are now logged out webpage
  webServer->on("/logged-out", HTTP_GET, [](AsyncWebServerRequest * request) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    Serial.println(logmessage);
    request->send_P(401, "text/html", logout_html, processor);
  });

  webServer->on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + + " " + request->url();

    if (checkUserWebAuth(request)) {
      logmessage += " Auth: Success";
      Serial.println(logmessage);
      request->send_P(200, "text/html", index_html, processor);
    } else {
      logmessage += " Auth: Failed";
      Serial.println(logmessage);
      return request->requestAuthentication();
    }

  });

  webServer->on("/reboot", HTTP_GET, [](AsyncWebServerRequest * request) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();

    if (checkUserWebAuth(request)) {
      request->send(200, "text/html", reboot_html);
      logmessage += " Auth: Success";
      Serial.println(logmessage);
    } else {
      logmessage += " Auth: Failed";
      Serial.println(logmessage);
      return request->requestAuthentication();
    }
  });

  webServer->on("/listfiles", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    if (checkUserWebAuth(request)) {
      logmessage += " Auth: Success";
      Serial.println(logmessage);
      request->send(200, "text/plain", listFiles(true));
    } else {
      logmessage += " Auth: Failed";
      Serial.println(logmessage);
      return request->requestAuthentication();
    }
  });

  webServer->on("/file", HTTP_GET, [](AsyncWebServerRequest * request) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    if (checkUserWebAuth(request)) {
      logmessage += " Auth: Success";

      if (request->hasParam("name") && request->hasParam("action")) {
        const char *fileName = request->getParam("name")->value().c_str();
        const char *fileAction = request->getParam("action")->value().c_str();

        logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url() + "?name=" + String(fileName) + "&action=" + String(fileAction);
        
        String slashFileName = "/" + String(fileName);

        if (!SPIFFS.exists(slashFileName)) {
          Serial.println(fileName);
          Serial.println(logmessage + " ERROR: file does not exist");
          request->send(400, "text/plain", "ERROR: file does not exist");
        } else {
          Serial.println(logmessage + " file exists");
          if (strcmp(fileAction, "download") == 0) {
            logmessage += " downloaded";
            request->send(SPIFFS, slashFileName, "application/octet-stream");
          } else if (strcmp(fileAction, "delete") == 0) {
            logmessage += " deleted";
            SPIFFS.remove(slashFileName);
            request->send(200, "text/plain", "Deleted File: " + String(slashFileName));
          } else if (strcmp(fileAction, "updateMega") == 0) {
            logmessage += " update Mega";
            request->send(200, "text/plain", "Update Mega: " + String(slashFileName));
          } else if (strcmp(fileAction, "updateEsp") == 0) {
            logmessage += " update Mega";
            request->send(200, "text/plain", "Update ESP32: " + String(slashFileName));
            File file = SPIFFS.open(String(slashFileName));
            if(!Update.begin(file.size())){
              Serial.println("Cannot do the update");
              return;
            };
            Update.writeStream(file);
            while(!Update.end()){
              vTaskDelay(10);
            }
            Serial.println("Successful update");  
            file.close();
            ESP.restart();
          }  else {
            logmessage += " ERROR: invalid action param supplied";
            request->send(400, "text/plain", "ERROR: invalid action param supplied");
          }
          Serial.println(logmessage);
        }
      } else {
        request->send(400, "text/plain", "ERROR: name and action params required");
      }
    } else {
      logmessage += " Auth: Failed";
      Serial.println(logmessage);
      return request->requestAuthentication();
    }
  });
}

void notFound(AsyncWebServerRequest *request) {
  String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
  Serial.println(logmessage);
  request->send(404, "text/plain", "Not found");
}

// used by server.on functions to discern whether a user has the correct httpapitoken OR is authenticated by username and password
bool checkUserWebAuth(AsyncWebServerRequest * request) {
  bool isAuthenticated = false;

  if (request->authenticate(THINGSBOARD_CLID, THINGSBOARD_PASS)) {
    Serial.println("authenticated via username and password");
    isAuthenticated = true;
  }
  return isAuthenticated;
}

// handles uploads to the filserver
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  // make sure authenticated before allowing upload
  if (checkUserWebAuth(request)) {
    String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
    Serial.println(logmessage);

    if (!index) {
      logmessage = "Upload Start: " + String(filename);
      // open the file on first call and store the file handle in the request object
      request->_tempFile = SPIFFS.open("/" + filename, "w");
      Serial.println(logmessage);
    }

    if (len) {
      // stream the incoming chunk to the opened file
      request->_tempFile.write(data, len);
      logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
      Serial.println(logmessage);
    }

    if (final) {
      logmessage = "Upload Complete: " + String(filename) + ",size: " + String(index + len);
      // close the file handle as the upload is now done
      request->_tempFile.close();
      Serial.println(logmessage);
      request->redirect("/");
    }
  } else {
    Serial.println("Auth: Failed");
    return request->requestAuthentication();
  }
}
//-------------------------------------------------------------------------------------------------------//
//-------------------------------------------GNSS Functions----------------------------------------------//
//-------------------------------------------------------------------------------------------------------//
void onRmcUpdate(nmea::RmcData const rmc)
{
  Serial.print("RMC ");

  if      (rmc.source == nmea::RmcSource::GPS)     Serial.print("GPS");
  else if (rmc.source == nmea::RmcSource::GLONASS) Serial.print("GLONASS");
  else if (rmc.source == nmea::RmcSource::Galileo) Serial.print("Galileo");
  else if (rmc.source == nmea::RmcSource::GNSS)    Serial.print("GNSS");

  if (rmc.is_valid)
  {
    CurrentTime.tm_sec = rmc.time_utc.second;
    CurrentTime.tm_min = rmc.time_utc.minute;
    CurrentTime.tm_hour = rmc.time_utc.hour;
    char rmcReadout[128] = {};
    snprintf(rmcReadout, 128, " %d:%d:%d.%d",rmc.time_utc.hour, rmc.time_utc.minute, rmc.time_utc.second, rmc.time_utc.microsecond);
    Serial.print(rmcReadout);
    snprintf(rmcReadout, 128, " : LON %f° | LAT %f° | VEL %f m/s | HEADING %f°",rmc.longitude,rmc.latitude,rmc.speed,rmc.course);
    Serial.print(rmcReadout);
  }
  Serial.println();
}

void onGgaUpdate(nmea::GgaData const gga)
{
  Serial.print("GGA ");

  if      (gga.source == nmea::GgaSource::GPS)     Serial.print("GPS");
  else if (gga.source == nmea::GgaSource::GLONASS) Serial.print("GLONASS");
  else if (gga.source == nmea::GgaSource::Galileo) Serial.print("Galileo");
  else if (gga.source == nmea::GgaSource::GNSS)    Serial.print("GNSS");

  char ggaReadout[128] = {};
  snprintf(ggaReadout, 128, " %d:%d:%d.%d",gga.time_utc.hour, gga.time_utc.minute, gga.time_utc.second, gga.time_utc.microsecond);
  Serial.print(ggaReadout);

  if (gga.fix_quality != nmea::FixQuality::Invalid)
  {
    CurrentTime.tm_sec = gga.time_utc.second;
    CurrentTime.tm_min = gga.time_utc.minute;
    CurrentTime.tm_hour = gga.time_utc.hour;
    snprintf(ggaReadout, 128, " : LON %f° | LAT %f | Num Sat. %d°",gga.longitude,gga.latitude, gga.num_satellites);
    Serial.print(ggaReadout);
    Serial.print(" | HDOP =  ");
    Serial.print(gga.hdop);
    Serial.print(" m | Altitude ");
    Serial.print(gga.altitude);
    Serial.print(" m | Geoidal Separation ");
    Serial.print(gga.geoidal_separation);
    Serial.print(" m");
  }

  Serial.println();
}
//-------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//