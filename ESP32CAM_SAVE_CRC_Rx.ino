//Inisialisasi Library yang akan digunakan
#include <SPI.h>
#include <SPIFFS.h>
#include <SD.h>
#include <LoRa.h>
#include <EEPROM.h>

// Inisialisasi pin LoRa
#define SS 15
#define RST 4
#define DIO0 2
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS 15

/Inisialisasi file penyimpanan Foto
#define FILE_PHOTO "/photo.jpg"  //path to the image in SPIFFS
#define EEPROM_SIZE 1

/Inisialisasi posisi nomor gambar
int EEPROM_position = 0;

const int chunk_size = 250;//ukuran paket (per 250 byte)
int total_packets;
int packet_number;
uint16_t received_crc; //uint16_t => ukuran setiap crc (16 bit atau 2 byte)
uint16_t calculated_crc;

//inisialisasi file gambar
String string_num;
File file;
File photo;
int EEPROM_count = 0;

//inisialisasi kebutuhan lora (bandwith dan spreading factor)
int bandwidth = 500000;
int spread_factor = 7;
String rssi;
String snr;
String path;
String timestamp;
int ack_retransmission_period = random(150, 200);//waktu untuk mentransmisikan ulang (ACK)
unsigned long start_millis;//waktu awal ketika esp dihidupkan
unsigned long current_millis;//waktu yang sedang berjalan
uint8_t chunk[250];//chunk=> payload yang dikumpulkan setiap 1 byte
byte current_packet;
int errorCount = 0;//untuk menghitung jumlah error

#define CRC_TYPE_CCITT 0//Tipe CRC (CCITT)
#define POLYNOMIAL_CCITT 0x1021//POlinomial
#define CRC_CCITT_SEED   0xFFFF//Nilai sisa

//Fungsi untuk menghitung crc setial byte
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

//Fungsi untuk menghitung crc dari kumpulan byte (Paket)
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

//Fungsi untuk menyimpan gambar baru
void new_photo() {
  if (EEPROM_count == 1000) {
    EEPROM_count = 0;
    EEPROM.begin(EEPROM_SIZE);
    EEPROM_position = EEPROM.read(0) + 1;
    EEPROM.write(0, EEPROM_position);
    EEPROM.commit();
  }
  path = "/picture" + String(EEPROM_position) + "_" + String(EEPROM_count) + ".jpg";  //path where image will be stored on SD card
  EEPROM_count++;
  Serial.printf("Picture file name: %s\n", path.c_str());
  photo = SD.open(path.c_str(), FILE_WRITE);  //save photo to SD card
  if (!photo) {
    Serial.println("Failed to open photo in writing mode");
  }
  file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);  //save current photo to SPIFFS
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  }
}

//Fungsi untuk mengakhiri penyimpanan gambar
void end_photo() {
  file.close();
  if (!photo) {
    Serial.println("Couldn't save to SD card");
  } else {
    photo.close();
    Serial.println("Picture saved to SD card");
  }
}

//Fungsi untuk mengulang transmisi apabila paket sudah diterima
void send_ack(const byte packet_number_rx) { 
  Serial.println("Sending ack for packet " + String(packet_number_rx));
  LoRa.setSignalBandwidth(bandwidth);
  LoRa.setSpreadingFactor(spread_factor);
  LoRa.beginPacket();
  LoRa.write(packet_number_rx);
  LoRa.endPacket();
}

void setup() {
  Serial.begin(115200);//Untuk komunikasi serial monitro

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  //HSPI is used by LoRa, VSPI is used by SD card reader
  SPIClass* hspi = NULL;
  hspi = new SPIClass(HSPI);//membuat hspi (komunikasi SPI) untuk Lora karena VSPI sudah digunakan SD card
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);//menyalakan HSPI
  LoRa.setSPI(*hspi);//set pin LoRa pada HSPI
  SPI.begin(18, 19, 23, 5);//konfigurasi VSP (Komunikasi SPI) untuk SD card
  LoRa.setPins(SS, RST, DIO0);//Konfigurasi pin untuk LoRa
  while (!LoRa.begin(433E6)) {//Memulai LoRa dengan frekuensi 433MHz
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0x6C);//Kata kunci yang digunakan LoRa dalam komunikasi
  //LoRa.enableCrc();
  LoRa.setSignalBandwidth(bandwidth);//Setting bandwith
  LoRa.setSpreadingFactor(spread_factor);//Setting spreading factor
  Serial.println("LoRa Initializing OK!");
  delay(500);
  if (!SD.begin()) {//Mengecek Kartu Sd
    Serial.println("SD Card Mount Failed");
  }
  EEPROM.begin(EEPROM_SIZE);//Memulai penomoran gambar
  EEPROM_position = EEPROM.read(0) + 1;
  EEPROM.write(0, EEPROM_position);
  EEPROM.commit();
  
  //clear SPIFFS
  SPIFFS.remove("/photo.jpg");
  Serial.println("Waiting for a new image");
  start_millis = millis();
}

void loop() {
  int packet_size = LoRa.parsePacket();//Pembacaan paket yang diterima
  if (packet_size) {
    while (LoRa.available()) {//Mengecek kondisi komunikasi lora masuk
      for (int i = 0; i < chunk_size; i++) {
        chunk[i] = LoRa.read();//pembacaan payload setiap byte hingga 250 (ukuran byte per baket)
      }
      uint16_t crcCCITT = RadioPacketComputeCrc(chunk, chunk_size, CRC_TYPE_CCITT);//Meghitung CRC pada payload di sisi penerima

      byte packet_info[4];//melihat paket info yang berisi 4 byte (Byte1 = nomor paket, Byte2 = jumlah paket, Byte3 = CRC high, byte4 = CRC Low)
      LoRa.readBytes(packet_info, 4);
      packet_number = (packet_info[0]); //Membaca nomor paket
      total_packets = (packet_info[1]); //Membaca total paket
      uint16_t crc_Received = (packet_info[2] + 256 * packet_info[3]); //Membaca crc high dan low (high dan low berukuran 2 Byte "16 Bit")=> Lora hanya dapat mengirim per byte sehingga dipecah
      //Menyimpan CRC Received (Kode CRC yang dikirimkan oleh transmitter)

      Serial.print("Perhitungan CRC: ");Serial.println(crcCCITT, HEX);//Menampilkan CRC perhitungan dari payload yang diterima
      Serial.print("Penerimaan CRC: ");Serial.println(crc_Received, HEX);//Menampilkan CRC yang dikirimkan oleh transmitter
      if(crcCCITT != crc_Received){//Mengecek CRC penerimaan dan perhitungan payload sama, jika tidak 
        Serial.println("ERRRROOOOORRRRRR!!!!!");//Menampilkan ERROR
        errorCount++;//Menambahkan jumlah error paket
      }
      Serial.print("Number of Packet Error: ");Serial.println(errorCount);//Menampilkan jumlah paket error
    }

    if (packet_number == 1){  //Apabila nomor paket 1 maka membuat gambar baru
      new_photo();
    }

    string_num = (String(packet_number)); //Konversi nomor paket ke string
    Serial.print("Got packet " + string_num + "/" + String(total_packets));//menampilkan nomor paket
    rssi = String(LoRa.packetRssi());
    snr = String(LoRa.packetSnr());
    Serial.println(" with RSSI: " + rssi + ", SNR: " + snr + " ,packet length= " + String(packet_size));//menampilkan nilai RSSI, SNR, dan Total paket
    if (!file) {//Jika file tidak dapat ditemukan / dilakukan penyimpanan (masalah sd card)
      Serial.println("Failed to open file in writing mode");
    } else {//jika ditemukan file untuk penyimpanan
      for (int i = 0; i < chunk_size; i++) {//menyimpan setiap 250 byte (pengaruh delay menyebabkan penyimpanan terganggu)
        file.write(chunk[i]);   //Menimpan di SPPIFS
        photo.write(chunk[i]);  //Menyimpan di SD card
        //Serial.print(String(chunk[i],HEX)); //for debugging purposes
      }
      //Serial.println("");
    }
    current_packet = packet_number;
    send_ack(current_packet);//mengirim konfirmasi apabila nomor paket telah diterima ke transmitter
    if (packet_number == total_packets) {  //ketika nomor paket sudah sesuai dengan jumlah paket
      current_packet = 0;//kembali ke 0
      errorCount = 0;
      end_photo();//mengakhiri penyimpanan gambar
    }
  }
  current_millis = millis();
  if (current_millis - start_millis >= ack_retransmission_period){//menunggu pengiriman paket selanjutnya
    ack_retransmission_period = random(150, 200);
    Serial.println("Sending acknowledgment again for packet "+String(current_packet));
    send_ack(current_packet);
    start_millis = current_millis;  //Menyimpan waktu awal
  }
}