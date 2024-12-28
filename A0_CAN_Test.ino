#include <esp32_can.h>

bool req_sent = false;
int resp_length = 0;
#define BUFFER_SIZE 20
byte vin_buffer[BUFFER_SIZE];
int buffer_pos = 0;

void printFrame(CAN_FRAME *message)
{
  Serial.print(message->id, HEX);
  if (message->extended) Serial.print(" X");
  Serial.print(" Len:");
  Serial.print(message->length, DEC);
  Serial.print(" ");
  
  for (int i = 0; i < message->length; i++) {
    Serial.print(message->data.byte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  Serial.println("Initializing...");

  CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5); // rx, tx
  CAN0.enable();
  CAN0.begin(500000);

  Serial.println("Ready!");

  CAN0.watchFor(0x780, 0xF80); // setup a filter for OBD messages
}

void loop() {
  CAN_FRAME message;

  if (!req_sent) {
    // Send service 09 Request vehicle information
    message.id = 0x7DF; // functional request CAN ID
    message.extended = false;
    message.length = 3;
    message.data.uint8[0] = 0x02; // single frame, length=2
    message.data.uint8[1] = 0x09; // service 09
    message.data.uint8[2] = 0x02; // InfoType 02 (VIN)
    CAN0.sendFrame(message);
    req_sent = true;
  }
  
  if (CAN0.read(message)) {
    // Look for FirstFrame message from ECU 0
    if (message.id == 0x7E8 && (message.data.uint8[0] & 0xF0) == 0x10 && message.data.uint8[2] == 0x49) {
      resp_length = ((message.data.uint8[0] & 0x0F) << 8) | message.data.uint8[1];
      Serial.print("Rx FF with length ");
      Serial.print(resp_length);
      Serial.print(" - ");

      // Begin copying VIN data to buffer
      buffer_pos = 0;
      if (resp_length > BUFFER_SIZE) resp_length = BUFFER_SIZE;
      
      for (int i = 2; i < message.length; i++) {
        Serial.write(message.data.uint8[i]);
        if (buffer_pos < resp_length) {
          vin_buffer[buffer_pos] = message.data.uint8[i];
          buffer_pos++;
        }
      }
      Serial.println();

      // Send flow control frame requesting ECU to send all remaining VIN bytes
      message.id = 0x7E0; // ECU #1 request CAN ID
      message.extended = false;
      message.length = 3;
      message.data.uint8[0] = 0x30; // flow control frame, FS=CTS
      message.data.uint8[1] = 0x00; // BS=0 (send all frames)
      message.data.uint8[2] = 0x00; // ST=0ms
      CAN0.sendFrame(message);
    }

    // Look for ConsecutiveFrame messages from ECU #1
    if (message.id == 0x7E8 && (message.data.uint8[0] & 0xF0) == 0x20) {
      byte sn = message.data.uint8[0] & 0x0F;
      Serial.print("Rx CF with SN ");
      Serial.print(sn);
      Serial.print(" - ");
      for (int i = 1; i < message.length; i++) {
        Serial.write(message.data.uint8[i]);

        if (buffer_pos < resp_length) {
          vin_buffer[buffer_pos] = message.data.uint8[i];
          buffer_pos++;
        }
      }
      Serial.println();
    }

    // Print VIN if fully received
    if (buffer_pos == resp_length) {
      Serial.print("VIN: ");
      for (int i = 3; i < resp_length; i++) {
        Serial.write(vin_buffer[i]);
      }
      Serial.println();
      buffer_pos = 0;
    }
  }
}
