#include <WiFi.h>
#include <WiFiUdp.h>
#include <DebugLog.h>
#include <opus.h>
#include "AudioKitHAL.h"
#include <ButterworthFilter.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <algorithm>


AsyncWebServer server(80);
WiFiUDP udpClient;
WiFiUDP udpPtt;
WiFiUDP udpSerial;

int audioPort_;
int pttPort_;
int serialPort_;


int pttPin = 36;
int pwrPin = 13;
int pwr_lastButtonState = LOW; 
int ptt_lastButtonState = LOW; 
int pwr_buttonState = 0; 
int ptt_buttonState = 0;
int outputPtt = 22;
int outputPwr = 19;


AudioKit audio;

#define SERIAL_BAUD_RATE      115200

#define AUDIO_SAMPLE_RATE     48000 //16000
#define AUDIO_OPUS_FRAME_MS   20
#define AUDIO_OPUS_BITRATE    44100 //9600
#define AUDIO_OPUS_COMPLEXITY 0
#define PACK_TIMEOUT 500
#define BUFFERSIZE 1024
#define BUFFER_SIZE 2048


OpusEncoder *opus_encoder_;
OpusDecoder *opus_decoder_;

TaskHandle_t audio_task_;
TaskHandle_t button_task_;
TaskHandle_t Serial_Bridge_;

int16_t *opus_samples_;
int opus_samples_size_;
uint8_t *opus_bits_;

uint8_t tx_buf[BUFFER_SIZE];
uint16_t tx_len=0;

uint8_t rx_buf[BUFFER_SIZE];
uint16_t rx_len=0;


// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "serverip";
const char* PARAM_INPUT_4 = "audioport";
const char* PARAM_INPUT_5 = "pttport";
const char* PARAM_INPUT_6 = "serialport";
const char* PARAM_INPUT_7 = "modeset";



//Variables to save values from HTML form
String ssid;
String pass;
String localip;
String serverip;
String audioport;
String pttport;
String serialport;
String modeset="CLIENT";

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* serveripPath = "/serverip.txt";
const char* audioportPath = "/audioport.txt";
const char* pttportPath = "/pttport.txt";
const char* serialportPath = "/serialport.txt";
const char* modesetPath = "/modeset.txt";

IPAddress localIP;

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- frite failed");
  }
}

// Initialize WiFi
bool initWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.println("Connecting to WiFi...");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("Failed to connect.");
      return false;
    }
  }
  localip = WiFi.localIP().toString();  
  Serial.println(WiFi.localIP());
  //Serial.println(localip1); 
  return true;
}

// Replaces placeholder with LED state value
String processor(const String& var) {
 

  if (var == "SSID"){
    return ssid;
  }  

  if (var == "LOCALIP"){
    return localip;
  }  

  if (var == "AUDIOPORT"){
    return audioport;
  } 

  if (var == "PTTPORT"){
    return pttport;
  }

  if (var == "SERIALPORT"){
    return serialport;
  }

  if (var == "MODE"){
    return modeset;
  } 

 
  return String();
}

void setup() {
  
  auto cfg = audio.defaultConfig(KitInputOutput);
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_LINE1;
  cfg.sample_rate = AUDIO_HAL_16K_SAMPLES; //08
  cfg.bits_per_sample = AUDIO_HAL_BIT_LENGTH_16BITS;
  audio.begin(cfg);
  audio.setVolume(50);
  
  pinMode(pttPin, INPUT);
  pinMode(pwrPin, INPUT);
  pinMode(outputPtt, OUTPUT);
  pinMode(outputPwr, OUTPUT);
  digitalWrite(outputPwr,LOW);
  digitalWrite(outputPtt,LOW);
  
  initSPIFFS();  
  Serial.begin(SERIAL_BAUD_RATE);
  Serial2.begin(19200, SERIAL_8N1, 18, 23); 

    // Load values saved in SPIFFS
  ssid = readFile(SPIFFS, ssidPath);
  pass = readFile(SPIFFS, passPath);
  serverip = readFile(SPIFFS, serveripPath);
  audioport = readFile (SPIFFS, audioportPath);
  pttport = readFile (SPIFFS, pttportPath);
  serialport = readFile (SPIFFS, serialportPath);
  modeset = readFile (SPIFFS, modesetPath);
  
  Serial.println(ssid);
  Serial.println(pass);
  Serial.println(serverip);
  Serial.println(audioport);
  Serial.println(pttport);
  Serial.println(serialport);
  Serial.println(modeset);
  audioPort_ = atoi(audioport.c_str());
  pttPort_ = atoi(pttport.c_str());
  serialPort_ = atoi(serialport.c_str());

  if(initWiFi()) {
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

    server.serveStatic("/", SPIFFS, "/");
    
    // Route to set GPIO state to HIGH
    server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request) {
     
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

    // Route to set GPIO state to LOW
    server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request) {
  
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

    // Route to set GPIO state to HIGH
    server.on("/tx", HTTP_GET, [](AsyncWebServerRequest *request) {
      //digitalWrite(pttPin, HIGH);
    
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

    // Route to set GPIO state to LOW
    server.on("/rx", HTTP_GET, [](AsyncWebServerRequest *request) {
      //digitalWrite(pttPin, LOW);
     
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

     server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
         // HTTP POST hostip value
          if (p->name() == PARAM_INPUT_3) {
            serverip = p->value().c_str();
            Serial.print("SERVER_IP Address set to: ");
            Serial.println(serverip);
            // Write file to save value
            writeFile(SPIFFS, serveripPath, serverip.c_str());
          }
          // HTTP POST port value
          if (p->name() == PARAM_INPUT_4) {
            audioport = p->value().c_str();
            Serial.print("Audio Port set to: ");
            Serial.println(audioport);
            // Write file to save value
            writeFile(SPIFFS, audioportPath, audioport.c_str());
          }
          // HTTP POST port value
          if (p->name() == PARAM_INPUT_5) {
            pttport = p->value().c_str();
            Serial.print("PTT Port set to: ");
            Serial.println(pttport);
            // Write file to save value
            writeFile(SPIFFS, pttportPath, pttport.c_str());
          }
          // HTTP POST port value
          if (p->name() == PARAM_INPUT_6) {
            serialport = p->value().c_str();
            Serial.print("Serial Port set to: ");
            Serial.println(serialport);
            // Write file to save value
            writeFile(SPIFFS, serialportPath, serialport.c_str());
          }
          // HTTP POST modeset value
          if (p->name() == PARAM_INPUT_7) {
            modeset = p->value().c_str();
            Serial.print("Mode set to: ");
            Serial.println(modeset);
            // Write file to save value
            writeFile(SPIFFS, modesetPath, modeset.c_str());
          }
        }
      }
    
      delay(1000);
      ESP.restart();
    });
    
    server.begin();
  
  if (!udpClient.begin(audioPort_)) {
    Serial.println("Failed to start UDP client");
    while (1); 
  }

  if (!udpPtt.begin(pttPort_)) {
    Serial.println("Failed to start PTT UDP server");
    while (1); 
  }

   if (!udpSerial.begin(serialPort_)) {
    Serial.println("Failed to start SERIAL UDP server");
    while (1); 
  }
  
  Serial.print("UDP client started on port ");
  Serial.println(audioPort_);

xTaskCreatePinnedToCore(&audio_task, "audio_task", 37000, NULL, 5, &audio_task_,1);
xTaskCreatePinnedToCore(&button_task, "button_task", 9000, NULL, 1, &button_task_,1);
xTaskCreatePinnedToCore(&Serial_Bridge,"Serial Bridge",5000,NULL,1,&Serial_Bridge_,1);

  
  
  }
  else {
    // Connect to Wi-Fi network with SSID and password
    Serial.println("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("IC706-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP); 

    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass.c_str());
          }
          
         
        }
      }
    
      delay(1000);
      ESP.restart();
    });
    server.begin();
  }
 
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

  while (true) {
    int packetSize = udpClient.parsePacket();
    if (packetSize) {
      udpClient.read(opus_bits_, BUFFERSIZE);
      int decoded_size = opus_decode(opus_decoder_, opus_bits_, packetSize, opus_samples_, opus_samples_size_, 0);
      
      audio.write(opus_samples_, sizeof(int16_t) * decoded_size);
      
    }

      
      audio.read(opus_samples_, sizeof(int16_t) * opus_samples_size_);
      int encoded_size = opus_encode(opus_encoder_, opus_samples_, opus_samples_size_, opus_bits_, BUFFERSIZE);
     
      udpClient.beginPacket(serverip.c_str(), audioPort_); 
      if(ptt_buttonState == LOW){
      udpClient.write(opus_bits_, encoded_size);
      udpClient.endPacket();
      }

      
   
   vTaskDelay(1); 
  }
}


void button_task(void *param) {
    const unsigned long longPressDuration = 500;
    unsigned long pwr_buttonPressStartTime = 0; 
    for(;;){
   // ptt_buttonState = digitalRead(pttPin);
    pwr_buttonState = digitalRead(pwrPin);

  if (pwr_buttonState != pwr_lastButtonState) {
    
    if(pwr_buttonState == LOW){
    pwr_buttonPressStartTime = millis();
    udpPtt.beginPacket(serverip.c_str(), pttPort_);
    udpPtt.printf("pwr_on");
    udpPtt.endPacket();
    Serial.println("POWER ON");
    } else {
    unsigned long buttonPressDuration = millis() - pwr_buttonPressStartTime;
    if (buttonPressDuration >= longPressDuration) {
    digitalWrite(outputPtt, HIGH); 
    delay(1000);
    digitalWrite(outputPtt, LOW
    );
    } 
    udpPtt.beginPacket(serverip.c_str(), pttPort_);
    udpPtt.printf("pwr_off");
    udpPtt.endPacket(); 
    Serial.println("POWER OFF");
    }
    
    pwr_lastButtonState = pwr_buttonState;
   }
 }
  vTaskDelay(10);
}




void loop() {
 
}



void Serial_Bridge(void *param) {
  for (;;) {
  int packetSize = udpSerial.parsePacket();
  if(packetSize>0) {
    udpSerial.read(tx_buf, packetSize);
    Serial2.write(tx_buf, packetSize); 
   
   }

  if(Serial2.available()) {
 
    while(1) 
    {
      if(Serial2.available()) {
       
        rx_buf[rx_len] = (char)Serial2.read(); 
        if(rx_len<BUFFER_SIZE-1) {
        rx_len++;
        } 
      }
      else {
        
        delayMicroseconds(PACK_TIMEOUT);
        if(!Serial2.available()) {
        break;
        }
      }
  }
  
  
    
    if(rx_len == 4 && (rx_buf[0] == 0xFE) && (rx_buf[1] == 0x0B)) 
    {
      rx_len = 0;
    }

    if((rx_len ==4) && (rx_buf[rx_len-4] == 0xFE) && (rx_buf[rx_len-3] == 0x00) && (rx_buf[rx_len-2] == 0x01) && (rx_buf[rx_len-1] == 0xFD)){
    ptt_buttonState == LOW;

   } else if ((rx_len ==4) && (rx_buf[rx_len-4] == 0xFE) && (rx_buf[rx_len-3] == 0x00) && (rx_buf[rx_len-2] == 0x00) && (rx_buf[rx_len-1] == 0xFD)){
    ptt_buttonState == HIGH;
  
   } 

    sendUDPData(rx_buf,rx_len);
  
  }
    
  
 }
}
const unsigned int MAX_PACKET_SIZE = 32;
void sendUDPData(const uint8_t* data, uint16_t dataSize) {
 
  for (unsigned int i = 0; i < dataSize; i += MAX_PACKET_SIZE) {
    unsigned int packetSize = std::min(MAX_PACKET_SIZE, static_cast<unsigned int>(dataSize - i));
    udpSerial.beginPacket(serverip.c_str(), serialPort_); 
    udpSerial.write(data + i, packetSize);
    Serial.write(data + i, packetSize); 
    udpSerial.endPacket();
    rx_len = 0;

  }
}
