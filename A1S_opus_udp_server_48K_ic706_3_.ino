#include <WiFi.h>
#include <WiFiUdp.h>
#include <DebugLog.h>
#include <opus.h>
#include "AudioKitHAL.h"


#define SERIAL_BAUD_RATE      115200
#define AUDIO_SAMPLE_RATE     48000 //16000
#define AUDIO_OPUS_FRAME_MS   20
#define AUDIO_OPUS_BITRATE    44100
//9600
#define AUDIO_OPUS_COMPLEXITY 0
#define PACK_TIMEOUT 500
#define BUFFERSIZE 1024  //audio data
#define BUFFER_SIZE 2048 //serial data

WiFiUDP udpServer;
WiFiUDP udpPtt;
WiFiUDP udpSerial;

const char *ssid = "Your ROUTER SSID";  // Your ROUTER SSID
const char *pw = "WiFi PASSWORD"; // and WiFi PASSWORD
const int localPort = 40000;
const int pttPort = 40010;
const int serialPort = 40020;

char freq0[8];
String vfoab;
String modeset;
char VfoData;
String ModeData;
String FreqData;
String freq1;
String freq2;
String freq3;
char menu[11];
String menudata;
char mch[2];
String mchdata;
char sig[1];
String sigdata;
String prevRigdata = "";

int outputPtt = 22;
int outputPwr = 19;
bool isWebSocketConnected;
bool ctlbtn;

int16_t *opus_samples_;
int opus_samples_size_;
uint8_t *opus_bits_;
uint8_t tx_buf[BUFFER_SIZE];
uint16_t tx_len=0;
uint8_t rx_buf[BUFFER_SIZE];
uint16_t rx_len=0;

uint8_t mbt[] = {0xFE, 0x01, 0x02, 0xFD};    //1
uint8_t f1bt[] = {0xFE, 0x01, 0x01, 0xFD};   //2
uint8_t f2bt[] = {0xFE, 0x02, 0x80, 0xFD};   //3
uint8_t f3bt[] = {0xFE, 0x02, 0x40, 0xFD};   //4
uint8_t modebt[] = {0xFE, 0x02, 0x08, 0xFD}; //5
uint8_t tsbt[] = {0xFE, 0x02, 0x04, 0xFD};   //6
uint8_t dispbt[] = {0xFE, 0x02, 0x20, 0xFD}; //7
uint8_t lockbt[] = {0xFE, 0x02, 0x10, 0xFD}; //8
uint8_t upbt[] = {0xFE, 0x02, 0x02, 0xFD};   //9
uint8_t downbt[] = {0xFE, 0x02, 0x01, 0xFD}; //10
uint8_t offbt[] = {0xFE, 0x06, 0x7A, 0xFD};  //11
uint8_t attbt[] = {0xFE, 0x01, 0x08, 0xFD};  //12
uint8_t tunebt[] = {0xFE, 0x01, 0x04, 0xFD}; //13

uint8_t ok1[] = {0xFE, 0x01, 0x00, 0xFD};
uint8_t ok2[] = {0xFE, 0x02, 0x00, 0xFD};

AudioKit audio;

OpusEncoder *opus_encoder_;
OpusDecoder *opus_decoder_;

void setup();
void audio_task(void *param);
void button_task(void *param);
void send_keepalive(void *param);
void rigscreen();
void PanelDecode();

TaskHandle_t audio_task_;
TaskHandle_t button_task_;
TaskHandle_t send_keepalive_;

void setup() {
 
  auto cfg = audio.defaultConfig(KitInputOutput);
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_LINE1;
  
  cfg.sample_rate = AUDIO_HAL_16K_SAMPLES;
  cfg.bits_per_sample = AUDIO_HAL_BIT_LENGTH_16BITS;
  audio.begin(cfg);
  audio.setVolume(100);

  pinMode(outputPtt, OUTPUT);
  pinMode(outputPwr, OUTPUT);
  digitalWrite(outputPwr,LOW);
  digitalWrite(outputPtt,LOW);
  Serial.begin(SERIAL_BAUD_RATE);
  Serial1.begin(19200, SERIAL_8N1, 18, 23); 
  Serial2.begin(115200, SERIAL_8N1, 5, 21); /
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  
  Serial.println("Starting UDP Server");  
  Serial.print("Local IP:");
  Serial.println(WiFi.localIP());
  
  Serial2.print("Local IP:");
  Serial2.println(WiFi.localIP());
  
  if (!udpServer.begin(localPort)) {
    Serial.println("Failed to start UDP server");
    while (1); 
  }

  if (!udpPtt.begin(pttPort)) {
    Serial.println("Failed to start PTT UDP server");
    while (1); 
  }

  if (!udpSerial.begin(serialPort)) {
  Serial.println("Failed to start SERIAL UDP server");
  while (1); 
  }
  
  Serial.print("UDP server started on port ");
  Serial.println(localPort);
  
  xTaskCreatePinnedToCore(&audio_task, "audio_task", 37000, NULL, 4, &audio_task_,1);
  xTaskCreatePinnedToCore(&button_task, "button_task", 30000, NULL, 1, &button_task_,1);
  xTaskCreatePinnedToCore(&send_keepalive,"keep alive",10000,NULL,1,&send_keepalive_,1);

}


void audio_task(void *param) {
  int encoder_error;
  opus_encoder_ = opus_encoder_create(AUDIO_SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP, &encoder_error);
  if (encoder_error != OPUS_OK) {
    LOG_ERROR("Failed to create OPUS encoder, error", encoder_error);
    return;
  }
  encoder_error = opus_encoder_init(opus_encoder_, AUDIO_SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP);
  if (encoder_error != OPUS_OK) {
    LOG_ERROR("Failed to initialize OPUS encoder, error", encoder_error);
    return;
  }
  opus_encoder_ctl(opus_encoder_, OPUS_SET_BITRATE(AUDIO_OPUS_BITRATE));
  opus_encoder_ctl(opus_encoder_, OPUS_SET_COMPLEXITY(AUDIO_OPUS_COMPLEXITY));
  opus_encoder_ctl(opus_encoder_, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));

  int decoder_error;
  opus_decoder_ = opus_decoder_create(AUDIO_SAMPLE_RATE, 1, &decoder_error);
  if (decoder_error != OPUS_OK) {
    LOG_ERROR("Failed to create OPUS decoder, error", decoder_error);
    return;
  } 

  opus_samples_size_ = (int)(AUDIO_SAMPLE_RATE / 1000 * AUDIO_OPUS_FRAME_MS);
  opus_samples_ = new int16_t[opus_samples_size_];
  opus_bits_ = new uint8_t[BUFFERSIZE];

  for (;;) { 
    int packetSize = udpServer.parsePacket();
    if (packetSize) {
      udpServer.read(opus_bits_, BUFFERSIZE);
      int decoded_size = opus_decode(opus_decoder_, opus_bits_, packetSize, opus_samples_, opus_samples_size_, 0);
      audio.write(opus_samples_, sizeof(int16_t) * decoded_size);
    }

      
      audio.read(opus_samples_, sizeof(int16_t) * opus_samples_size_);
      int encoded_size = opus_encode(opus_encoder_, opus_samples_, opus_samples_size_, opus_bits_, BUFFERSIZE);
     // if(digitalRead(outputPtt) == 0){ //ptt_on ->rig AF off
      udpServer.beginPacket(udpServer.remoteIP(), udpServer.remotePort()); 
      udpServer.write(opus_bits_, encoded_size);
      udpServer.endPacket();
     // }
  }
}

void send_keepalive(void *param)
{
   for(;;){
  
     uint8_t msg[] = {0xFE,0x0B,0x00,0xFD}; 
     Serial1.write(msg,4);
     vTaskDelay(150/ portTICK_RATE_MS);
    
  }
}

void button_task(void *param) {

 for(;;){
  int packetSize = udpPtt.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udpPtt.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
  
      if (strcmp(packetBuffer, "pwr_on") == 0) {
        digitalWrite(outputPwr, HIGH);
        //Serial.println("GPIO 19 set to HIGH");
      } else if (strcmp(packetBuffer, "pwr_off") == 0) {
        digitalWrite(outputPwr, LOW);
        //Serial.println("GPIO 19 set to LOW");
      }
    }
  }
 }
}
const unsigned long SERIAL_DELAY1 = 100;
void loop() {

  // if there’s data available, read a packet
  int packetSize = udpSerial.parsePacket();
  if(packetSize>0) {
    udpSerial.read(tx_buf, packetSize);
    
   if((packetSize ==4) && (tx_buf[packetSize-4] == 0xFE) && (tx_buf[packetSize-3] == 0x00) && (tx_buf[packetSize-2] == 0x01) && (tx_buf[packetSize-1] == 0xFD)){
    digitalWrite(outputPtt, HIGH);
 
   } else if ((packetSize ==4) && (tx_buf[packetSize-4] == 0xFE) && (tx_buf[packetSize-3] == 0x00) && (tx_buf[packetSize-2] == 0x00) && (tx_buf[packetSize-1] == 0xFD)){
    digitalWrite(outputPtt, LOW);
 
   } 
  Serial1.write(tx_buf, packetSize);  
 
  }

  if(Serial1.available()) {
 
    while(1) {
      if(Serial1.available()) {
        rx_buf[rx_len] = (char)Serial1.read(); 
        if(rx_len<BUFFER_SIZE-1) {
          rx_len++;
        }
      } 
     else {
       
       delayMicroseconds(PACK_TIMEOUT);
        if(!Serial1.available()) {
        break;
        }
      }
    }
    
    PanelDecode();   
    rigscreen();
 
    udpSerial.beginPacket(udpSerial.remoteIP(), udpSerial.remotePort()); 
    udpSerial.write(rx_buf, rx_len);
  
    udpSerial.endPacket();
    rx_len = 0;
   }

    if (Serial.available()) { 
    char receivedChar = Serial.read(); 
    Serial1.write(receivedChar); 
    }
  
}




void PanelDecode()
{
  
   if((rx_buf[0] == 0xFE) && (rx_buf[1] == 0x60))
     {
     freq0[0] = rx_buf[6];
     freq0[1] = rx_buf[7];
     freq0[2] = rx_buf[8];
     
     freq0[3] = rx_buf[9];
     freq0[4] = rx_buf[10];
     freq0[5] = rx_buf[11];
     
     freq0[6] = rx_buf[12];
     freq0[7] = rx_buf[13];
     
     menu[0] = rx_buf[24];
     menu[1] = rx_buf[25];

     menu[2] = rx_buf[26];
     menu[3] = rx_buf[27];
     menu[4] = rx_buf[28];

     menu[5] = rx_buf[30];
     menu[6] = rx_buf[31];
     menu[7] = rx_buf[32];

     menu[8] = rx_buf[34];
     menu[9] = rx_buf[35];
     menu[10] = rx_buf[36];

     mch[0] = rx_buf[15];
     mch[1] = rx_buf[16];

     sig[0] = rx_buf[22];
    
     if((rx_buf[19] == 0x21) || (rx_buf[19] == 0x25)) //25
     {
     vfoab = 'B';
     } 
     else if((rx_buf[19] == 0x41)|| (rx_buf[19] == 0x25)) //45
          {
          vfoab = 'A';
          } else if((rx_buf[19] == 0x11)){
            vfoab = 'M';
          }

     if((rx_buf[17] == 0xC8) && (rx_buf[18] == 0x02))
     {
      modeset = "FM  ";
     }
     if((rx_buf[17] == 0xC8) && (rx_buf[18] == 0x06))
     {
      modeset = "WFM ";
     }
     if(((rx_buf[17] == 0xC1) || (rx_buf[17] == 0xC9))  && (rx_buf[18] == 0x00)) //c9
     {
      modeset = "LSB ";
     }
     if(((rx_buf[17] == 0xC0) || (rx_buf[17] == 0xC8)) && (rx_buf[18] == 0x80)) //c8
     {
      modeset = "USB ";
     }
     if(((rx_buf[17] == 0xC0) || (rx_buf[17] == 0xC8)) && (rx_buf[18] == 0x40)) //c8
     {
      modeset = "CW  ";
     }
     if(((rx_buf[17] == 0xC0) || (rx_buf[17] == 0xC8)) && (rx_buf[18] == 0x60)) //c8
     {
      modeset = "CW-R";
     }
     if(((rx_buf[17] == 0xC0) || (rx_buf[17] == 0xC8)) && (rx_buf[18] == 0x10)) //c8
     {
      modeset = "RTTY";
     }
     if((rx_buf[17] == 0xC8) && (rx_buf[18] == 0x08))
     {
      modeset = "AM  ";
     }
  }
}


void rigscreen(){
    
      String str(freq0);
      String dot = ".";
      FreqData = freq0; 
      freq1 = FreqData.substring(0,3);
      freq2 = FreqData.substring(3,6);
      freq3 = FreqData.substring(6,8);
      String freq = freq1 + dot + freq2 + dot + freq3;
      String sub(menu);
      menudata = menu;
      String mchstr(mch);
      mchdata = mch;
      String rigdata;
      String index = "RIGDATA";
      rigdata = index + freq + vfoab + modeset + menu + mchdata;
            
       if (rigdata != prevRigdata) {
        Serial.println(rigdata);
        Serial2.println(rigdata);
        prevRigdata = rigdata; 
    }
  
}
