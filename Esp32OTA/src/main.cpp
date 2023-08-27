#include <Arduino.h>

#define SP_MAVLINK Serial
#define SP_DEBUG Serial2
//============GLOBAL CONST =============
#define BLE_RECV_BUFF_LEN (15)

//LED管脚
#define K1 32
//============GLOBAL CONST END =============

//===========  COMMOM ASSETS  ============
char read_ssid[BLE_RECV_BUFF_LEN],read_pwd[BLE_RECV_BUFF_LEN];
//========================================

//=============================== OTA ===============================
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
#endif

#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

AsyncWebServer server(80);

bool g_ota_is_running = false;
void OTA_setup(void) {
  if(g_ota_is_running == false)
  {
    g_ota_is_running = true;  //仅启动一次

      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! This is a sample response.");
    });

    AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
    server.begin();
    SP_DEBUG.println("HTTP server started");
  }
}

//=============================== OTA END ===============================

//=============================== DATA LINK =============================
//============ includes ===============
#include <WiFi.h>
#include <WiFiUdp.h>

#include <BLEDevice.h>
#include <BLEServer.h>
//#include <BLEUtils.h>
#include <BLE2902.h>

#include "EEPROM.h"

//================================= EEPROM ===============================================
// Instantiate eeprom objects with parameter/argument names and sizes
#define EEPROM_SIZE (36)
#define EEPROM_SSID_OFFSET (0)
#define EEPROM_PWD_OFFSET (BLE_RECV_BUFF_LEN+3)

void setup_eeprom() {
  delay(1000);
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    //SP_DEBUG.println("failed to initialise EEPROM"); 
    delay(1000000);
  }
}

void eeprom_read()
{
  for (int i = EEPROM_SSID_OFFSET; i < BLE_RECV_BUFF_LEN+EEPROM_SSID_OFFSET; i++)
  {
    read_ssid[i-EEPROM_SSID_OFFSET] = byte(EEPROM.read(i));
  }

  for (int i = EEPROM_PWD_OFFSET; i < BLE_RECV_BUFF_LEN+EEPROM_PWD_OFFSET; i++)
  {
    read_pwd[i-EEPROM_PWD_OFFSET] = byte(EEPROM.read(i));
  }
}

void eeprom_save(char *ssid, char* pwd)
{
  // Write: Variables ---> EEPROM stores
  for (int i = EEPROM_SSID_OFFSET; i < BLE_RECV_BUFF_LEN+EEPROM_SSID_OFFSET; i++)
  {
    //read_ssid[i-EEPROM_SSID_OFFSET] = byte(EEPROM.read(i));
    EEPROM.write(i, ssid[i-EEPROM_SSID_OFFSET]);
  }

  for (int i = EEPROM_PWD_OFFSET; i < BLE_RECV_BUFF_LEN+EEPROM_PWD_OFFSET; i++)
  {
    EEPROM.write(i, pwd[i-EEPROM_PWD_OFFSET]);
  }
  EEPROM.commit();
  // Read back
  eeprom_read();
  // check
  if(strcmp(read_ssid,ssid)!=0 || strcmp(read_pwd,pwd)!=0)
  {
    //SP_DEBUG.println(ssid);
    //SP_DEBUG.println(read_ssid);
    //SP_DEBUG.println(pwd);
    //SP_DEBUG.println(read_pwd);
    //SP_DEBUG.println("Failed to read back ssid or/and pwd.");
  }
  else
  {
    SP_DEBUG.println("Save Succeed.");
  }
}

//================================= EEPROM END ===============================================

//================================= BLE ==================================================
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
BLECharacteristic * pRxCharacteristic_ssid, *pRxCharacteristic_pwd;
bool deviceConnected = false;
bool oldDeviceConnected = false;
//uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX_SSID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  //SSID
#define CHARACTERISTIC_UUID_RX_PWD "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   //PWD
#define CHARACTERISTIC_UUID_TX "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"     //NOTIFY

char ssid_string[BLE_RECV_BUFF_LEN];
char pwd_string[BLE_RECV_BUFF_LEN];

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        if (pCharacteristic == pRxCharacteristic_ssid)
        {
          //ssid Received Value:
          memset(ssid_string,0,BLE_RECV_BUFF_LEN);
          for (int i = 0; i < rxValue.length(); i++)
          {
            ssid_string[i] = rxValue[i];
          }
        }
        else if (pCharacteristic == pRxCharacteristic_pwd)
        {
          //pwd Received Value:
          memset(pwd_string,0,BLE_RECV_BUFF_LEN);
          for (int i = 0; i < rxValue.length(); i++)
          {
            pwd_string[i] = rxValue[i];
          }
        }
      }
      //SP_DEBUG.print("ssid/pwd:");
      //SP_DEBUG.print(ssid_string);
      //SP_DEBUG.print("/");
      //SP_DEBUG.println(pwd_string);
      
      eeprom_save(ssid_string, pwd_string);
    }
};


void setup_BLE() {
  //SP_DEBUG.begin(115200);

  // Create the BLE Device
  BLEDevice::init("Drone Transciver");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic_ssid = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX_SSID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic_ssid->setCallbacks(new MyCallbacks());

  pRxCharacteristic_pwd = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX_PWD,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic_pwd->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising, Waiting a client connection to notify...
  pServer->getAdvertising()->start();

}
//================================= BLE END ==================================================

//================================= WIFI =====================================================
//const char* ssid = "XXX";
//const char* password = "XXX";
//WiFi连接标志
bool wifi_connected,wifi_last_connect_state;
//IP地址最后一段
#define LAST_IP_SECTOR 255

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
//const char * udpAddress = "192.168.100.114";
IPAddress staticLocalIp,staticRemoteIp;
const int udpPort_bit = 3333;
const int udpPort_data = 3334;
const int udpPort_rc = 3335;

//The udp library class
WiFiUDP udp_BIT;
WiFiUDP udp_Data;
WiFiUDP udp_RC;

IPAddress remoteIp(WiFi.localIP());

//BIT上报分频
uint16_t bit_prescaler = 0;
const uint16_t BIT_PRESCALER = 300;  //分频计数
uint32_t udp_recv_cnt = 0;  //udp接收到的字节计数
uint32_t uart_recv_cnt = 0; //SP_MAVLINK接收到的字节计数
uint32_t udp_rc_recv_cnt = 0; //RC控制指令接收字节计数

//串口2接收缓冲区
#define USART2_RECV_BUF_SZIE (2048)
uint8_t SP_MAVLINK_recv_buf[USART2_RECV_BUF_SZIE];

//UDP_DATA端口接收缓冲区
#define UDP_DATA_RECV_BUF_SIZE (2048)
uint8_t udp_data_recv_buf[UDP_DATA_RECV_BUF_SIZE];

void StartUDPService()
{
   
  //initializes the UDP state
  //This initializes the transfer buffer
  udp_BIT.begin(WiFi.localIP(),udpPort_bit);
  udp_Data.begin(WiFi.localIP(),udpPort_data);
  udp_RC.begin(WiFi.localIP(),udpPort_rc);
}

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info) {
  wifi_connected = true;
  StartUDPService();
  OTA_setup();
  SP_DEBUG.println("Successfully connected to Access Point");
}

void Get_IPAddress(WiFiEvent_t event, WiFiEventInfo_t info) {
  SP_DEBUG.println("WIFI is connected!");
  SP_DEBUG.println("IP address: ");
  SP_DEBUG.println(WiFi.localIP());
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  wifi_connected = false;
  SP_DEBUG.println("Disconnected from WIFI access point");
  SP_DEBUG.print("WiFi lost connection. Reason: ");
  SP_DEBUG.println(info.wifi_sta_disconnected.reason);
  SP_DEBUG.println("Reconnecting...");
  WiFi.begin(ssid_string, pwd_string);
}

void setup_wifi() {
  //WiFi连接标志
  wifi_connected = false;

  //串口初始化
  SP_MAVLINK.setRxBufferSize(USART2_RECV_BUF_SZIE);
  SP_MAVLINK.begin(115200);

  //WiFi断开连接
  WiFi.disconnect(true);
  delay(1000);

  //WiFi事件注册
  WiFi.onEvent(Wifi_connected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(Get_IPAddress, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  //WiFi启动
  WiFi.begin(ssid_string, pwd_string);
  SP_DEBUG.println("Waiting for WIFI network...");
}

//================================= WIFI END=====================================================
void setup() {
  //LED初始化
  pinMode(K1 , OUTPUT);
  SP_DEBUG.begin(115200);

  SP_DEBUG.println("Version 1.02, Characteristics: OTA, Double Serials, BIT");

  //rtc_wdt_protect_off();    // 看门狗写保护关闭，关闭后可以喂狗
  //rtc_wdt_enable();         // 启动看门狗
  //rtc_wdt_set_time(RTC_WDT_STAGE0, 800);  // 设置看门狗超时 800ms，超时重启

  setup_BLE();
  setup_eeprom();
  eeprom_read();
  strcpy(ssid_string,read_ssid);
  strcpy(pwd_string,read_pwd);
  SP_DEBUG.print("ssid/pwd:");
  SP_DEBUG.print(ssid_string);
  SP_DEBUG.print("/");
  SP_DEBUG.println(pwd_string);
  setup_wifi();
}

void loop() {
  ///BLE
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      //delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      //start advertising
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
  // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }

  // WiFi
  //连接成功则配置一次静态IP
  if(wifi_last_connect_state == false && wifi_connected == true)
  {
    remoteIp = WiFi.localIP();
    remoteIp[3] = LAST_IP_SECTOR;
  }
  //wifi_last_connect_state = wifi_connected;
  //only send data when connected
  if (wifi_connected) {
    //DATA UDP Process
    {
      //check UART2 for data
      if (SP_MAVLINK.available()) {
        size_t len = SP_MAVLINK.available();
        uart_recv_cnt+=len; //数据长度记录,BIT
        len = len>USART2_RECV_BUF_SZIE?USART2_RECV_BUF_SZIE:len;  //防止超出缓冲区长度
        SP_MAVLINK.readBytes(SP_MAVLINK_recv_buf, len);
        udp_Data.beginPacket(remoteIp, udpPort_data);
        udp_Data.write(SP_MAVLINK_recv_buf, len);
        udp_Data.endPacket();
      }
      //send to UART2
      int packetSize = udp_Data.parsePacket(); //获取当前队首数据包长度
      if (packetSize)                     //如果有数据可用
      {
        udp_Data.read(udp_data_recv_buf, packetSize); //读取当前包数据
        udp_recv_cnt+=packetSize; //数据长度记录,BIT
        SP_MAVLINK.write(udp_data_recv_buf, packetSize);
      }
    }
    
    //RC UDP Process
    {
      //send to UART1
      int packetSize = udp_RC.parsePacket(); //获取当前队首数据包长度
      if (packetSize)                     //如果有数据可用
      {
        udp_RC.read(udp_data_recv_buf, packetSize); //读取当前包数据
        udp_rc_recv_cnt+=packetSize; //数据长度记录,RC
        SP_DEBUG.write(udp_data_recv_buf, packetSize);
      }
    }

    //BIT分频上报
    if(bit_prescaler++>BIT_PRESCALER)
    {
      bit_prescaler=0;
      //BIT UDP Send a packet
      udp_BIT.beginPacket(remoteIp, udpPort_bit);
      udp_BIT.printf("T: %.2f ", float(millis() / 1000));
      //udp_BIT.printf("NetState: %d\n", WiFi.status());
      udp_BIT.printf("udp_recv_cnt: %ld uart_recv_cnt: %ld udp_rc_recv_cnt: %ld\n", udp_recv_cnt,uart_recv_cnt,udp_rc_recv_cnt);
      udp_BIT.endPacket();
      digitalWrite(K1, 1 - digitalRead(K1));
      //SP_DEBUG.print("RSSI: ");
      //SP_DEBUG.println(WiFi.RSSI());

      //BLE 打印
      if (deviceConnected) {
        uint8_t _localip[2];
        _localip[0]=WiFi.localIP()[2];
        _localip[1]=WiFi.localIP()[3];
        char __temp_str[17]={0};
        sprintf(__temp_str,"%d.%d.%d.%d",WiFi.localIP()[0],WiFi.localIP()[1],WiFi.localIP()[2],WiFi.localIP()[3]);
        pTxCharacteristic->setValue((uint8_t*)__temp_str, 17);
        pTxCharacteristic->notify();
        //delay(1000); // bluetooth stack will go into congestion, if too many packets are sent
      }
    
    }
    delay(10);
  }
  else
  {
    delay(100);
    digitalWrite(K1, 1 - digitalRead(K1));
  }
}

//=============================== DATA LINK END =============================
