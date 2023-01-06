#include <Wire.h>

// RFD Transmit Example Code
// Sending is this simple. Works as any other serial communication device would

//#include <RelayXBee.h>

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

#define RFDSerial Serial
#define RFD_BAUD 115200
const int MPU=0x68;
uint16_t packet[6];
uint16_t termination = 0x0000;

void setup() { 
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  RFDSerial.begin(RFD_BAUD); 
}

void loop() {
//    Wire.beginTransmission(MPU);
//    Wire.write(0x3B);  
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU,12,true); 
//    for (uint8_t i = 0; i < 6; i++) { 
//      packet[i]=Wire.read()<<8|Wire.read();
//      Serial.print(packet[i]);
//      RFDSerial.println(packet[i]);
//    }   
////    for(uint8_t i = 0; i < 6; i++) {
////      RFDSerial.println(packet[i]);
////    }
//
//    //RFDSerial.print(termination);
//    delay(2000);

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read(); 
    
//  uint16_t AcX_copy = AcX;
    uint8_t cache1, cache2;

    cache1 = (AcX >> 8) & 0x00FF;
    cache2 = AcX & 0xFF;
    Serial.write(cache1); // Send the upper byte first
    Serial.write(cache2); // Send the lower byte

    Serial .write(0x00);    

    cache1 = (AcY >> 8) & 0x00FF;
    cache2 = AcY & 0xFF;
    Serial.write(cache1); // Send the upper byte first
    Serial.write(cache2); // Send the lower byte

    Serial .write(0x00);

    cache1 = (AcZ >> 8) & 0x00FF;
    cache2 = AcZ & 0xFF;
    Serial.write(cache1); // Send the upper byte first
    Serial.write(cache2); // Send the lower byte

    Serial .write(0x00);

    cache1 = (GyX >> 8) & 0x00FF;
    cache2 = GyX & 0xFF;
    Serial.write(cache1); // Send the upper byte first
    Serial.write(cache2); // Send the lower byte

    Serial .write(0x00);

    cache1 = (GyY >> 8) & 0x00FF;
    cache2 = GyY & 0xFF;
    Serial.write(cache1); // Send the upper byte first
    Serial.write(cache2); // Send the lower byte

    Serial .write(0x00);

    cache1 = (GyZ >> 8) & 0x00FF;
    cache2 = GyZ & 0xFF;
    Serial.write(cache1); // Send the upper byte first
    Serial.write(cache2); // Send the lower byte

    
    Serial .write(0x00);
    Serial.write(0x00);

//  Serial.write (0x61);
//  Serial.println(((packet[0] >> 8) & 0x00FF), HEX); // Send the upper byte first
//  Serial.println((packet[0] & 0xFF),HEX); // Send the lower byte
//  Serial.println(packet[0]);
//  Serial.println(" ");
//  Serial.print(" | Y = "); Serial.print(AcY);
//  Serial.print(" | Z = "); Serial.println(AcZ); 
//  
//  Serial.print("Gyroscope: ");
//  Serial.print("X = "); Serial.print(GyX);
//  Serial.print(" | Y = "); Serial.print(GyY);
//  Serial.print(" | Z = "); Serial.println(GyZ);
//  Serial.println(" ");
////
//  Serial.write(AcX);
  delay(2000);

  
}
