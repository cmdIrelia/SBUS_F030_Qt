#include <WiFi.h>
#include <WiFiUdp.h>

//LED管脚
#define K1 2

const char* ssid = "wifiwifi";
const char* password = "qweqweqwe";
//const char* ssid = "Haorenjie";
//const char* password = "peter930319";

//WiFi连接标志
bool connected;

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char * udpAddress = "192.168.100.114";
const int udpPort_bit = 3333;
const int udpPort_data = 3334;
const int udpPort_rc = 3335;

//The udp library class
WiFiUDP udp_BIT;
WiFiUDP udp_Data;
WiFiUDP udp_RC;

//BIT上报分频
uint16_t bit_prescaler = 0;
const uint16_t BIT_PRESCALER = 300;  //分频计数
uint32_t udp_recv_cnt = 0;  //udp接收到的字节计数
uint32_t uart_recv_cnt = 0; //serial2接收到的字节计数
uint32_t udp_rc_recv_cnt = 0; //RC控制指令接收字节计数

//串口2接收缓冲区
#define USART2_RECV_BUF_SZIE (2048)
uint8_t serial2_recv_buf[USART2_RECV_BUF_SZIE];

//UDP_DATA端口接收缓冲区
#define UDP_DATA_RECV_BUF_SIZE (2048)
char udp_data_recv_buf[UDP_DATA_RECV_BUF_SIZE];

void StartUDPService()
{
  //initializes the UDP state
  //This initializes the transfer buffer
  udp_BIT.begin(WiFi.localIP(),udpPort_bit);
  udp_Data.begin(WiFi.localIP(),udpPort_data);
  udp_RC.begin(WiFi.localIP(),udpPort_rc);
}

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info) {
  connected = true;
  StartUDPService();
  Serial.println("Successfully connected to Access Point");
}

void Get_IPAddress(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WIFI is connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  connected = false;
  Serial.println("Disconnected from WIFI access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Reconnecting...");
  WiFi.begin(ssid, password);
}

void setup() {
  //WiFi连接标志
  connected = false;

  //串口初始化
  Serial.begin(115200);
  Serial2.begin(57600);
  Serial2.setRxBufferSize(USART2_RECV_BUF_SZIE);

  //LED初始化
  pinMode(K1 , OUTPUT);

  //WiFi断开连接
  WiFi.disconnect(true);
  delay(1000);

  //WiFi事件注册
  WiFi.onEvent(Wifi_connected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(Get_IPAddress, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  //WiFi启动
  WiFi.begin(ssid, password);
  Serial.println("Waiting for WIFI network...");
}

void loop() {
  //only send data when connected
  if (connected) {
    //DATA UDP Process
    {
      //check UART2 for data
      if (Serial2.available()) {
        size_t len = Serial2.available();
        uart_recv_cnt+=len; //数据长度记录,BIT
        len = len>USART2_RECV_BUF_SZIE?USART2_RECV_BUF_SZIE:len;  //防止超出缓冲区长度
        Serial2.readBytes(serial2_recv_buf, len);
        udp_Data.beginPacket(udpAddress, udpPort_data);
        udp_Data.write(serial2_recv_buf, len);
        udp_Data.endPacket();
      }
      //send to UART2
      int packetSize = udp_Data.parsePacket(); //获取当前队首数据包长度
      if (packetSize)                     //如果有数据可用
      {
        udp_Data.read(udp_data_recv_buf, packetSize); //读取当前包数据
        udp_recv_cnt+=packetSize; //数据长度记录,BIT
        Serial2.write(udp_data_recv_buf, packetSize);
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
        Serial.write(udp_data_recv_buf, packetSize);
      }
    }

    //BIT分频上报
    if(bit_prescaler++>BIT_PRESCALER)
    {
      bit_prescaler=0;
      //BIT UDP Send a packet
      udp_BIT.beginPacket(udpAddress, udpPort_bit);
      udp_BIT.printf("Time: %.2f ", float(millis() / 1000));
      udp_BIT.printf("NetState: %d\n", WiFi.status());
      udp_BIT.printf("udp recv: %ld uart recv: %ld rc recv: %ld", udp_recv_cnt,uart_recv_cnt,udp_rc_recv_cnt);
      udp_BIT.endPacket();
      digitalWrite(K1, 1 - digitalRead(K1));
      //Serial.print("RSSI: ");
      //Serial.println(WiFi.RSSI());
    }
    delay(20);
  }
  else
  {
    delay(100);
    digitalWrite(K1, 1 - digitalRead(K1));
  }
}
