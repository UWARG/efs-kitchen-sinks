#define RFDSerial Serial1
#define RFD_BAUD 115200  // This is the baud all of our RFDs communicate at. They can be changed to something else, but you have to make sure these always match up.

uint8_t packet[100]; // Make sure the size of this array is larger than your largest data string will be. In this case, 100 characters is the max.

void setup() {
  Serial.begin(115200);
  RFDSerial.begin(RFD_BAUD);
}

void loop() {
  if(RFDSerial.available()>0){                  // Checks for any incoming bytes
    //delay(10);// Bytes will be received one at a time unless you add a small delay so the buffer fills with your message
    int incomingBytes = RFDSerial.available();  // Checks number of total bytes to be read
    Serial.println(incomingBytes);              // Just for testing to see if delay is sufficient to receive all bytes.
    for(int i=0; i<incomingBytes; i++)
    {
      packet[i] = RFDSerial.read();             // Reads bytes one at a time and stores them in a character array.
    }
//    Serial.print(packet[0], HEX);
//    Serial.print(packet[1], HEX);Serial.print("\n");

    uint16_t result[10];
    result[0] = (((packet[0] << 8) | packet[1]) & 0XFFFF);
    result[1] = (((packet[2] << 8) | packet[3]) & 0XFFFF);
//    result[2] = (((packet[4] << 8) | packet[5]) & 0XFFFF);
//    result[3] = (((packet[6] << 8) | packet[7]) & 0XFFFF);
//    result[4] = (((packet[8] << 8) | packet[9]) & 0XFFFF);
//    result[5] = (((packet[10] << 8) | packet[11]) & 0XFFFF);
//    result[6] = (((packet[12] << 8) | packet[13]) & 0XFFFF);
//    result[7] = (((packet[14] << 8) | packet[15]) & 0XFFFF);
//    result[8] = (((packet[16] << 8) | packet[17]) & 0XFFFF);
//    result[9] = (((packet[18] << 8) | packet[19]) & 0XFFFF);


  for(int i = 0; i < 10; i++) {
    Serial.write(result[i]);
  }
  }
}
