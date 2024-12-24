//Inisialisasi library yang digunakan
#include <Arduino.h>
#include "FS.h"                                  //SD Card ESP32
#include "SD_MMC.h"                              //SD Card ESP32
#include "soc/soc.h"                             //disable brownout problems
#include "soc/rtc_cntl_reg.h"                    //disable brownout problems
#include "driver/rtc_io.h"
#include "esp_camera.h"
#include <EEPROM.h>            // read and write from flash memory
#include <SPI.h>
#include <LoRa.h>

//Inisialisasi pin LoRa
#define NSS 12                //select on LoRa device
#define RST 15
#define SCK 4
#define MISO 13
#define MOSI 2
#define DIO0 -1//pin yang tidak digunakan = -1
#define REDLED 33             //pin number for ESP32CAM on board red LED, set logic level low for on

//Inisialisasi pin pada modul camera esp32 cam
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//Inisialisasi penyimpanan gambar
#define EEPROM_SIZE 1
int pictureNumber = 0;        //number of picture taken, set to 0 on reset

//Inisialisasi kebutuhan LoRa
static int chunk_size = 250;  //Ukuran setiap chunk (250 byte)
long int bandwidth = 500000; //inisialisasi bandwith
int spread_factor = 7;  //7-12 //inisialisasi sf
int total_packets = 0;
unsigned long start_millis = 0;
unsigned long current_millis = 0;
int ack_wait_timeout = 60000; //waktu tunggu konfirmasi penerimaan paket oleh receiver
int SF_find_timeout = 300; //Waktu tunggu pencarim spreading factor 

byte total_packets_tx[1];//inisialisai nomor paket yang akan dikirimkan
byte packet_number_tx[1];//inisialisasi total paket yang akan dikirimkan
uint8_t crc_array[2];//inisialisasi crc yang akan dikirimkan

#define CRC_TYPE_CCITT 0//tipe crc
#define POLYNOMIAL_CCITT 0x1021//polinomial
#define CRC_CCITT_SEED   0xFFFF//nilai sisa

camera_config_t config;//konfigurasi modul kamera esp32 cam

//perhitungan CRC16 setiap 1 byte
uint16_t ComputeCrc(uint16_t crc, uint8_t data, uint16_t polynomial) {
    for (int i = 0; i < 8; i++) {
        if (((crc & 0x8000) >> 8) ^ (data & 0x80)) {
            crc <<= 1;
            crc ^= polynomial;
        } else {
            crc <<= 1;
        }
        data <<= 1;
    }
    return crc;
}

//Perhitungan CRC16 setiap chunk (250 Byte / 1 paket)
uint16_t RadioPacketComputeCrc(uint8_t* buffer, uint8_t bufferLength, uint8_t crcType) {
    uint16_t crc;
    uint16_t polynomial;

    // Select polynomial and seed based on CRC type
    polynomial = POLYNOMIAL_CCITT;
    crc = CRC_CCITT_SEED;

    // Compute CRC for each byte in the buffer
    for (int i = 0; i < bufferLength; i++) {
        crc = ComputeCrc(crc, buffer[i], polynomial);
    }

    // Return CRC based on type
    if (crcType != CRC_TYPE_CCITT) {
        return crc;
    } else {
        return (uint16_t)(~crc);
    }
}

//pengiriman paket
void send_chunk(int packet_number, camera_fb_t* fb) {
  uint8_t chunk[chunk_size];//insialisasi chunk
  memcpy(chunk, (fb->buf) + (chunk_size * (packet_number - 1)), chunk_size);//mengambil nilai dari gambar yang telah dipotret
  Serial.println("Sending packet: " + String(packet_number));
  LoRa.beginPacket();//memulai loRa
  for (int j = 0; j < chunk_size; j++) {
    LoRa.write(chunk[j]);//mengirim byte per byte payload
  }
  
  uint16_t crcCCITT = RadioPacketComputeCrc(chunk, chunk_size, CRC_TYPE_CCITT);//menghitung CRC per chunnk 
  crc_array[0] = crcCCITT % 256;//membagi CRC16 menjadi per byte (uint8 / 8 bit) high dan low
  crc_array[1] = crcCCITT / 256;

  LoRa.write(packet_number_tx[0]);//mengirim byte nomor paket
  LoRa.write(total_packets_tx[0]);//mengirimm byte total paket
  LoRa.write(crc_array[0]);//mengirim byte crc16 high
  LoRa.write(crc_array[1]);//mengirim byte crc16 low
  LoRa.endPacket();//mengakhiri LoRa
}

void setup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);//menghidupkan fungsi brownout
  redFlash(4, 125, 125);//konfigurasi led flash
  rtc_gpio_hold_dis(GPIO_NUM_4);//menonaktifkan pin 4
  rtc_gpio_hold_dis(GPIO_NUM_12);
  
  pinMode(2, INPUT_PULLUP);
  digitalWrite(NSS, HIGH);//menghidupkan pin NSS
  pinMode(NSS, OUTPUT);

  Serial.begin(115200);//memulai komunikasi serial monitor
  Serial.println();

  if (!initMicroSDCard()){//mengecek Sd card
    Serial.println(F("****************************"));
    Serial.println(F("ERROR - SD Card Mount Failed"));
    Serial.println(F("****************************"));
    return;
  }
  else{
    Serial.println(F("SD Card OK"));//jika terdapat sd card
  }

  if (!configInitCamera()){//mengecek konfigurasi modul kamer esp32 cam
    Serial.println(F("Camera config failed"));//jika modul kamera tidak ada
    return;
  }
  start_millis = millis();
  takePhotoSave();//menjalankan fungsi takePhotoSave (mengambil gambar)
}

void loop(){
}

//fungsi untuk mencari Spreading factor yang sesuai apabila tidak ada penerima yang konfirmasi penerimaan paket
int findSF() {
  start_millis = millis();
  Serial.println("Trying SF 7");
  LoRa.setSpreadingFactor(7);
  while (true) {
    int packet_size = LoRa.parsePacket();
    if (packet_size) {
      Serial.println("SF is 7");
      return 1;
    }
    current_millis = millis();
    if (current_millis - start_millis >= SF_find_timeout) {
      break;
    }
  }
  start_millis = millis();
  Serial.println("Trying SF 8");
  LoRa.setSpreadingFactor(8);
  while (true) {
    int packet_size = LoRa.parsePacket();
    if (packet_size) {
      Serial.println("SF is 8");
      return 1;
    }
    current_millis = millis();
    if (current_millis - start_millis >= SF_find_timeout) {
      break;
    }
  }
  start_millis = millis();
  Serial.println("Trying SF 9");
  LoRa.setSpreadingFactor(9);
  while (true) {
    int packet_size = LoRa.parsePacket();
    if (packet_size) {
      Serial.println("SF is 9");
      return 1;
    }
    current_millis = millis();
    if (current_millis - start_millis >= SF_find_timeout) {
      break;
    }
  }
  start_millis = millis();
  Serial.println("Trying SF 10");
  LoRa.setSpreadingFactor(10);
  while (true) {
    int packet_size = LoRa.parsePacket();
    if (packet_size) {
      Serial.println("SF is 10");
      return 1;
    }
    current_millis = millis();
    if (current_millis - start_millis >= SF_find_timeout) {
      break;
    }
  }
  start_millis = millis();
  Serial.println("Trying SF 11");
  LoRa.setSpreadingFactor(11);
  while (true) {
    int packet_size = LoRa.parsePacket();
    if (packet_size) {
      Serial.println("SF is 11");
      return 1;
    }
    current_millis = millis();
    if (current_millis - start_millis >= SF_find_timeout) {
      break;
    }
  }
  start_millis = millis();
  Serial.println("Trying SF 12");
  LoRa.setSpreadingFactor(12);
  while (true) {
    int packet_size = LoRa.parsePacket();
    if (packet_size) {
      Serial.println("SF is 12");
      return 1;
    }
    current_millis = millis();
    if (current_millis - start_millis >= SF_find_timeout) {
      break;
    }
  }
  return 0;
}

//fungsi untuk mencari bandwith yang sesuai apabila tidak ada penerima yang konfirmasi penerimaan paket
void find_BW() {
  Serial.println("I have got no acknowledgement from the receiver in "+String(ack_wait_timeout/1000)+" seconds.\nMaybe it switched to a different bandwidth and spreading factor. \nNow I will attemp to try these values.");
  while (true) {
    LoRa.setSignalBandwidth(7800);
    Serial.println("Trying BW 7800");
    if (findSF() == 1) {
      Serial.println("BW is 7800");
      return;
    }
    LoRa.setSignalBandwidth(10400);
    Serial.println("Trying BW 10400");
    if (findSF() == 1) {
      Serial.println("BW is 10400");
      return;
    }
    LoRa.setSignalBandwidth(15600);
    Serial.println("Trying BW 15600");
    if (findSF() == 1) {
      Serial.println("BW is 15600");
      return;
    }
    LoRa.setSignalBandwidth(20800);
    Serial.println("Trying BW 20800");
    if (findSF() == 1) {
      Serial.println("BW is 20800");
      return;
    }
    LoRa.setSignalBandwidth(31250);
    Serial.println("Trying BW 31250");
    if (findSF() == 1) {
      Serial.println("BW is 31250");
      return;
    }
    LoRa.setSignalBandwidth(41700);
    Serial.println("Trying BW 41700");
    if (findSF() == 1) {
      Serial.println("BW is 41700");
      return;
    }
    LoRa.setSignalBandwidth(62500);
    Serial.println("Trying BW 62500");
    if (findSF() == 1) {
      Serial.println("BW is 62500");
      return;
    }
    LoRa.setSignalBandwidth(125000);
    Serial.println("Trying BW 125000");
    if (findSF() == 1) {
      Serial.println("BW is 125000");
      return;
    }
    LoRa.setSignalBandwidth(250000);
    Serial.println("Trying BW 250000");
    if (findSF() == 1) {
      Serial.println("BW is 250000");
      return;
    }
    LoRa.setSignalBandwidth(500000);
    Serial.println("Trying BW 500000");
    if (findSF() == 1) {
      Serial.println("BW is 500000");
      return;
    }
  }
}

//menunggu konfirmasi penerimaan paket dari penerima
int wait_for_ack(int packet_number){
  start_millis = millis();
  while (true) {
    current_millis = millis();
    if (current_millis - start_millis >= ack_wait_timeout) {
      find_BW();
      start_millis = current_millis;
    }
    int packet_size = LoRa.parsePacket();
    if (packet_size) {
      byte packet_confirmation_rx[1];
      while (LoRa.available()) {
        LoRa.readBytes(packet_confirmation_rx, 1);
        int packet_confirmation = packet_confirmation_rx[0];
        if (packet_confirmation != packet_number) {
          Serial.println("Expected confirmation for packet " + String(packet_number) + ", got confirmation for packet " + String(packet_confirmation) + ". Sending packet " + String(packet_number) + " again...");
          return 0;
        } else {
          Serial.println(String(packet_confirmation) + " OK!");
          return 1;
        }
      }
    }
  }
}

//fungsi untuk konfigurasi penyesuaian led flash (pin flash tersebut mengganggu kinerja lora)
void redFlash(uint16_t flashes, uint16_t ondelaymS, uint16_t offdelaymS){
  uint16_t index;
  pinMode(REDLED, OUTPUT);                    //setup pin as output
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(REDLED, LOW);
    delay(ondelaymS);
    digitalWrite(REDLED, HIGH);
    delay(offdelaymS);
  }
  pinMode(REDLED, INPUT);                     //setup pin as input
}

//fungsi untuk konfigurasi LoRa
bool setupLoRaDevice(){
  SPI.begin(SCK, MISO, MOSI, NSS);
  LoRa.setPins(NSS, RST, DIO0);
  if (LoRa.begin(433E6)){
    Serial.println(F("LoRa device found"));
  }else{
    Serial.println(F("LoRa Device error"));
    return false;
  }
  LoRa.setSyncWord(0x6C);
  //LoRa.enableCrc();
  LoRa.setSignalBandwidth(bandwidth);
  LoRa.setSpreadingFactor(spread_factor);
  Serial.println("LoRa Initializing OK!");
  return true;
}

//Fungsi untuk mengecek sd card
bool initMicroSDCard(){
  if (!SD_MMC.begin("/sdcard", true)){
    return false;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE){
    Serial.println(F("Unknown SD card type"));
    return false;
  }
  return true;
}

//Fungsi untuk konfigurasi kamera esp32 cam (brightness, berwarna, dll)
bool configInitCamera(){
  Serial.println(F("Initialising the camera module "));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;      //YUV422,GRAYSCALE,RGB565,JPEG

  //Select lower framesize if the camera doesn't support PSRAM
   if (psramFound()) {
    config.frame_size = FRAMESIZE_HD;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);   //Initialize the Camera
  if (err != ESP_OK){
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 2);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 2); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 450);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 1);            // 0 = disable , 1 = enable
  s->set_wpc(s, 0);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  return true;
}

//fungsi untuk memotret dan menyimpan gambar yang terpotret
void takePhotoSave(){
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;//penambahan 1 nomor untuk penomoran gambar

  // lokasi untuk pengimpanan gambar foto dan penamaan file gambar
  String path = "/pic" + String(pictureNumber) + ".jpg";
  Serial.print("Taking picture number ");
  Serial.print(pictureNumber);
  Serial.print("... ");

  //Memotret melalui gambar (fb = payload gambar)
  camera_fb_t * fb = esp_camera_fb_get();
  delay(1000);
  if (!fb){//ketika kamera tidak dapat memotret foto
    Serial.println("Camera capture failed");
    return;
  }
  delay(1000);

  // Menyimpan gambar di SD card
  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if (!file){//ketika tidak dapat menyimpan file
    Serial.println("Failed to open file in writing mode");
  }else{
    file.write(fb->buf, fb->len); // fb->buf = payload gambar, fb->len = ukuran payload gambar
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();//mengakhiri penyimpanan gambar
  SD_MMC.end();

  if (setupLoRaDevice()){//mengecek konfigurasi LoRa
    int size = fb->len;  //Mengambil ukuran payload gambar
    Serial.println("Size of picture is: " + String(size) + " bytes");//menampilkan ukuran gambar
    total_packets = (size / chunk_size) + 1;//membagi ukuran gambar per 250 byte
    Serial.println("Total number of packets to transmit: " + String(total_packets));//menampilkan ukuran gambar yang telah dibagi menjadi paket
    total_packets_tx[0] = total_packets;//menyimpan total paket pada tipe data byte
    for (int i = 1; i <= total_packets; i++) {  //Mengirimkan setiap paket
      packet_number_tx[0] = i;//packet number = nomor packet, misal ada 10 total paket berarti ada 1,2,3,4,5,6,7,8,9,10 nomor paket
      send_chunk(i, fb);//menjalankan fungsi pengiriman gambar

      while (wait_for_ack(i) == 0) {//menunggu konfirmasi penerimaan gambar oleh receiver
        send_chunk(i, fb);//mengirim ulang apabila nomor paket tetap 0
      }
    }
  }else{
    Serial.println("LoRa device not available");
  }
  esp_camera_fb_return(fb);//mengembalikan kekondisi semula modul kamera esp32 cam, agar dapat memotret lagi

  delay(2000);
  Serial.println("Going to sleep now");
  delay(2000);
  esp_deep_sleep_start();//mematikan esp32cam ketika transmisi terselesaikan
  Serial.println("This will never be printed");
}