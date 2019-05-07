// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>

/* for feather32u4 */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#define KART_KILL 5
#define FEATHER_SDA 2
#define FEATHER_SCL 3

/* for feather m0
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 3
*/

/* for shield
  #define RFM95_CS 10
  #define RFM95_RST 9
  #define RFM95_INT 7
*/

/* Feather 32u4 w/wing
  #define RFM95_RST     11   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

/* Feather m0 w/wing
  #define RFM95_RST     11   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     6    // "D"
*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  pinMode(KART_KILL, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(KART_KILL, LOW);

  Wire.begin(FEATHER_I2C_ADDRESS); //join I2C bus with address
  Wire.onReceive(receive_event);   //register event
  while (!Wire) {
    delay(1);
  }
  
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  delay(100);

  Serial.println("Feather LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(15, false); // want to conserve as much power as possible
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    Serial.println("Waiting for reply...");
    if (rf95.waitAvailableTimeout(1000))
    {
      // Should be a reply message for us now
      if (rf95.recv(buf, &len))
      {
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        if (buf[0] == 0x02 && buf[len - 1] == 0x03) 
        { 
          if (buf[1] == 0xff) 
          {
            //send kill signal pulse to emergency brake for 0.5 seconds
            digitalWrite(KART_KILL, HIGH);
            delay(500);   
            digitalWrite(KART_KILL, LOW);
            Serial.println("Kart e-brake engaged!");    
          }
          else if (buf[1] == 0x01) 
          {
             //do something with I2C
          }

        }
      }
      else
      {
        Serial.println("Receive failed");
      }
    }
    else
    {
      Serial.println("No reception, is there a transmitter around?");
    }
  }
}



void receive_event(int num_bytes) {
  while (1 < Wire.available()) { //loop through all but last
    char c = Wire.read();        //receive byte as char
    Serial.print(c);             //print character
  }
  int x = Wire.read();           //receive byte as int
  Serial.println(x);             //print int
}
