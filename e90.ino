#include <SPI.h>
#include <MFRC522.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Secrets.h"

#define RST_PIN 5 // D1
#define SDA_PIN 15 // D8
#define IRQ_PIN 4 // D2
#define SOL_PIN 10 // SD3     
#define ID 1 // the id of the microcontroller, is hardcoded 
#define UID_LEN 4 // length of the uid we will take.

MFRC522 mfrc522(SDA_PIN, RST_PIN);   // Create MFRC522 instance.
MFRC522::MIFARE_Key key; // what does this do?? comment later to see if shit breaks

// TODO: need to add a thing that alerts public safety when bike isn't there that
// is expected to be there. how often will you want to check bikes?

// TODO: how to handle spurious interrupt on startup?
const char* inTopic = "bikeshare/1";
const char* outTopic = "bikeshare/feed/1";
const byte failureMessage[1] = {0};

// ip address of home comp. also consider not hardcoding this?? or secrets?
IPAddress mqtt_server(ip[0], ip[1], ip[2], ip[3]);

/* THINGS THAT ARE NOT CONSTANT AND WILL CHANGE. */
byte regVal = 0x7F; 
WiFiClient espClient;
PubSubClient client(espClient);
int state = 0;
int checkInFlag = 0; // set when ready to check in
int flag = 0; // when flag is set, interrupt
byte expectedUid[4] = {0, 0, 0, 0}; // set this to zero normally, 0x8B 0xFE 0x00 0x89 else

/* FUNCTION STUBS */
void activateRec(MFRC522 mfrc522); // retrigger receive on mfrc522
void clearInt(MFRC522 mfrc522); // interrupt func
void setup_wifi_and_mqtt(); // sets up connections
void cb(char* topic, byte* payload, unsigned int payload_len); // callback
void reconnect();
void setFlag();
void activateSolenoid();
boolean readCard();

void setup() {
  Serial.begin(115200); // Initialize serial communications with the PC
  while (!Serial);      // wait for serial port open
  SPI.begin();          // Init SPI bus

  setup_wifi_and_mqtt();
  
  mfrc522.PCD_Init(); // Init MFRC522 card

  // interrupt needs to be an input because card is slave, 
  // microcontroller is master.
  pinMode(IRQ_PIN, INPUT_PULLUP);
  pinMode(SOL_PIN, OUTPUT);

  regVal = 0xA0; //rx irq
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);

  /*Activate the interrupt*/
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), setFlag, FALLING);
  digitalWrite(SOL_PIN, LOW); // ensure solenoid in lock state
  reconnect();
  Serial.println(F("End setup"));
}

void loop() {
  if (flag && checkInFlag) { 
    Serial.println("flag raised");
      if (readCard()) { checkInFlag = 0; }  // only say checkInFlag okay when correct id read.
    flag = 0;
  }

  if (!client.loop()) { reconnect(); }

  activateRec(mfrc522); // The receiving block needs regular retriggering
  delay(100); // don't want it to read too often...
}

/* returns: true if expected uid read, false if unexpected uid read.
 * does: triggered only when expecting bike check in. will publish check-in message
 *       on success, or will publish failure message on failure. */
boolean readCard() {
  Serial.println("Card detected!"); 
  mfrc522.PICC_ReadCardSerial(); // read the tag data
  dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
  
  clearInt(mfrc522);
  mfrc522.PICC_HaltA(); // tells the card to chill

  reconnect();
  if (mfrc522.uid.size != UID_LEN) { // only accept correct length

    // a length 0 card size is usually a problem with the tag swipe,
    // not an incorrect card.
    if (mfrc522.uid.size != 0) { 
      client.publish(outTopic, failureMessage, 1);
      Serial.println("Bad length");  
    }
    return false; // TODO: alert the user on failure, to be done on app side.
  } 
  for(int i = 0; i < UID_LEN; i++) { // make sure its the same.
    if(mfrc522.uid.uidByte[i] != expectedUid[i]) { 
      client.publish(outTopic, failureMessage, 1);
      Serial.println("\nID does not match.");
      return false; 
    }
  }

  if (!client.publish(outTopic, mfrc522.uid.uidByte, mfrc522.uid.size)) {
    // TODO: how to handle this case? no way to alert the user, since publishing
    // isn't apparently working...
    Serial.println("Publish failed"); // TODO: how to handle this case?
  }
  
  client.disconnect();
  state = 0; // bike has been returned.
  return true;
}

void setFlag() { flag = 1; }

// sends MFRC522 the needed commands to activate the reception
void activateRec(MFRC522 mfrc522) {
  // put 0x26 in the 64-byte FIFO buffer -- command to make card be read-ready
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);

  // writes transceive command (0000 1100) in CommandReg -- transmits data
  // from FIFO buffer to antenna and activate receiver automatically after 
  // transmission.
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);

  /* writes 1000 0111 into BitFramingReg[7:0]
   * [7]: StartSend -- when value = 1, starts the transmission of data. 
   *      only valid with transceive
   * [6:4]: RxAlign -- when 0, LSB of received bit at 0, 2nd received bit at 0 
   *        (controls endian) 
   * [3]: reserved
   * [2:0]: TxLastBits -- defines # of bits of the last byte that will be 
   *        transmitted. 111 indicates all transmitted. */
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

// clears interrupt bits.
void clearInt(MFRC522 mfrc522) { mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F); }

void setup_wifi_and_mqtt() {
  delay(10); // ?? if u take this away things break??? idk why

  WiFi.mode(WIFI_STA);  // set into station mode (client)
  WiFi.begin(ssid, password);

  int timeout_cntr = 0;
  while (WiFi.status() != WL_CONNECTED) { // wait for connect
    delay(500);
    Serial.print(".");
    timeout_cntr++;
    if (timeout_cntr > 240) { // if it's been longer than 2 min
      Serial.println("Connection timeout. Don't keep your hopes up.");
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWifi connected!");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // mqtt setup.
  client.setServer(mqtt_server, 1883); // 1883 is the default port
  client.setCallback(cb);
}

/* function that gets called automatically upon message receive.
 * params: char* topic - the name of the topic we've subscribed to
 *         byte* payload - contents of message received
 *         unsigned int payload_len - length of message */
void cb(char* topic, byte* payload, unsigned int payload_len) {
  Serial.print("Message received from topic [");
  Serial.print(topic); // we should only ever be subscribed to 1 topic, so eh.
  Serial.print("] ");

  if (payload_len == 1 && payload[0] == 1) { // check out
    Serial.println("checking out");
    activateSolenoid();
    state = 1; // bike's not there anymore. 
  } else if (payload_len == UID_LEN) { // check in, and give 10 seconds to check in
    Serial.println("Received the UID: ");
    dump_byte_array(payload, UID_LEN);
    memcpy(expectedUid, payload, UID_LEN); // dest, source, len
    checkInFlag = 1;
    activateSolenoid();
  }
}

// silent success.
void reconnect() {
  while (!client.connected()) { // Loop until we're reconnected
    if (client.connect("ESP8266Client")) { // Attempt to connect
      client.subscribe(inTopic); 
    } else {
       Serial.print("MQTT connection failed, rc=");
       Serial.print(client.state());
       Serial.println(" trying again in 5 seconds");
       // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void activateSolenoid() {
  digitalWrite(SOL_PIN, HIGH); // unlock
  delay(10000); // wait 10 sec
  digitalWrite(SOL_PIN, LOW); // relock.
}

void dump_byte_array(byte *buffer, byte bufferSize) {
  Serial.println(bufferSize);
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

