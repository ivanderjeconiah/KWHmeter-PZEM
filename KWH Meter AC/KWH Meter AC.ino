#define BLYNK_PRINT Serial// Uncomment for debugging 
#define BLYNK_TEMPLATE_ID "TMPLiy9Icizd"
#define BLYNK_DEVICE_NAME "Power Meter"
#define BLYNK_AUTH_TOKEN "H69ULqGnrCSkdx1wbz303k8IM4mABVOW"

#include <BlynkSimpleEsp8266.h>
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>

#include <SoftwareSerial.h>  //  ( NODEMCU ESP8266 )
SoftwareSerial pzem(D5, D6); //D5:RO/RX  & D6:DI/TX

#define MAX485_DE      D1
#define MAX485_RE_NEG  D2

static uint8_t pzemSlaveAddr = 0x01;
ModbusMaster Master1;

double Tegangan, Arus, Daya_Aktif, Energy_Aktif, Frekuensi, Faktor_Daya; 
uint8_t result;  
uint16_t data[6];

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {
  
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(9600);
  pzem.begin(9600);

  // Modbus slave ID 1
  Master1.begin(pzemSlaveAddr, pzem);
  
  // Callbacks allow us to configure the RS485 transceiver correctly
  Master1.preTransmission(preTransmission);
  Master1.postTransmission(postTransmission);
  Blynk.begin(BLYNK_AUTH_TOKEN, "1", "1sampai8");

}


void pzemdata(){

    //node.clearResponseBuffer(); 
    result = Master1.readInputRegisters(0x0000, 10);
    if (result == Master1.ku8MBSuccess){
    Tegangan              = (Master1.getResponseBuffer(0x00) / 10.0f);
    Arus                  = (Master1.getResponseBuffer(0x01) / 1000.000f);
    Daya_Aktif            = (Master1.getResponseBuffer(0x03) / 10.0f);
    Energy_Aktif          = (Master1.getResponseBuffer(0x05) / 1000.0f);
    Frekuensi             = (Master1.getResponseBuffer(0x07) / 10.0f);
    Faktor_Daya           = (Master1.getResponseBuffer(0x08) / 100.0f);

    Serial.print("VOLTAGE:           ");   Serial.println(Tegangan);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(Arus, 3);        //  A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(Daya_Aktif);     //  W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(Energy_Aktif, 3);// kWh
    Serial.print("FREQUENCY:         ");   Serial.println(Frekuensi);      // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(Faktor_Daya);
    Serial.println("====================================================");  

    Blynk.virtualWrite(V0,          Tegangan);
    Blynk.virtualWrite(V1,          Arus);
    Blynk.virtualWrite(V4,          Daya_Aktif);
    Blynk.virtualWrite(V5,          Energy_Aktif);
    Blynk.virtualWrite(V3,          Frekuensi);
    Blynk.virtualWrite(V2,          Faktor_Daya);

    delay(1000);
          
    }
    else{
      Serial.println("Komunikasi gagal!,Periksa terminasi serial");
    }
}

void loop() {
  Blynk.run();
  pzemdata();
}



BLYNK_WRITE(V6){
  if(param.asInt()==1){ 
    uint16_t u16CRC = 0xFFFF;                       /* declare CRC check 16 bits*/
    static uint8_t resetCommand = 0x42;             /* reset command code*/
    uint8_t slaveAddr =pzemSlaveAddr;
    u16CRC = crc16_update(u16CRC, slaveAddr);
    u16CRC = crc16_update(u16CRC, resetCommand);
    preTransmission();                              /* trigger transmission mode*/                
    pzem.write(slaveAddr);                          /* send device address in 8 bit*/
    pzem.write(resetCommand);                       /* send reset command */
    pzem.write(lowByte(u16CRC));                    /* send CRC check code low byte  (1st part) */
    pzem.write(highByte(u16CRC));                   /* send CRC check code high byte (2nd part) */ 
    delay(10);
    postTransmission();                             /* trigger reception mode*/
    delay(100);
  }
}
