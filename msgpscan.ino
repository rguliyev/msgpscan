// https://github.com/merkur2k/MSCan_Gauge

#include <HardwareSerial.h>
#include <ESP32-TWAI-CAN.hpp>
#include <ESP32Time.h>

// Default for ESP32
#define CAN_RX		21
#define CAN_TX		22

#define myCANid   10 // CAN ID of this unit
#define msCANid   0 // CAN ID of the Megasquirt (should almost always be 0)

#define CANtimeout 25 // ms, default is 1000

#define DEBUG     false

const PROGMEM uint8_t ClearConfig[] = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98 };
const PROGMEM uint8_t GPGLLOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
const PROGMEM uint8_t GPGSVOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
const PROGMEM uint8_t GPVTGOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47 };
const PROGMEM uint8_t GPGSAOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };
const PROGMEM uint8_t GPGGAOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24 };
const PROGMEM uint8_t GPRMCOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40 };
const PROGMEM uint8_t Navrate10hz[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12 };
const PROGMEM uint8_t NavPvtOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC };
const PROGMEM uint8_t NavPOSLLHOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9 };
const PROGMEM uint8_t NavSTATUSOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0 };

const PROGMEM uint8_t NavPOSLLHOn[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE };
const PROGMEM uint8_t NavSTATUSOn[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5 };
const PROGMEM uint8_t NavPvtOn[] = { 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1};

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };
const unsigned char NAV_POSLLH_HEADER[] = { 0x01, 0x02 };
const unsigned char NAV_STATUS_HEADER[] = { 0x01, 0x03 };
const unsigned char NAV_PVT_HEADER[] = { 0x01, 0x07 };

// GPS globals
bool validTime;              // Time Validity
bool gpsFixOk;                 // Position and velocity valid 
unsigned char numSV;         // Number of satellites used in Nav Solution
long lon;                    // Longitude (deg)
long lat;                    // Latitude (deg)
unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)
unsigned short year;         // Year (UTC)
unsigned char month;         // Month, range 1..12 (UTC)
unsigned char day;           // Day of month, range 1..31 (UTC)
unsigned char hour;          // Hour of day, range 0..23 (UTC)
unsigned char minute;        // Minute of hour, range 0..59 (UTC)
unsigned char second;        // Seconds of minute, range 0..60 (UTC)
int8_t latdeg, londeg, altk;
unsigned char latmin, lonmin;
unsigned int latmmin, lonmmin, heading;
unsigned short gSpeed;
short altm;

enum _ubxMsgType {
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS,
  MT_NAV_PVT
};

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

struct NAV_STATUS {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned char gpsFix;
  char flags;
  char fixStat;
  char flags2;
  unsigned long ttff;
  unsigned long msss;
};

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)
  unsigned short year;         // Year (UTC)
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char minute;        // Minute of hour, range 0..59 (UTC)
  unsigned char second;        // Seconds of minute, range 0..60 (UTC)

  char valid;                  // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  char flags;                  // Fix Status Flags
  char flags2;     // reserved
  unsigned char numSV;         // Number of satellites used in Nav Solution

  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)

  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long heading;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headingAcc;    // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision

  short flags3;                // Additional flags
  unsigned char reserved0[4];  // Reserved
  long headVeh;                // Heading of vehicle (2-D)
  short magDec;                 // Magnetic declination
  unsigned short magAcc;       // Magnetic declination accuracy
};


union UBXMessage {
  NAV_POSLLH navPosllh;
  NAV_STATUS navStatus;
  NAV_PVT navPvt;
};


// pack/unpack the Megasquirt extended message format header
typedef struct msg_packed_int {
  unsigned char b0;
  unsigned char b1;
  unsigned char b2;
  unsigned char b3;
} msg_packed_int;

typedef struct msg_bit_info {
  unsigned int spare:2;
  unsigned int block_h:1;
  unsigned int block:4;
  unsigned int to_id:4;
  unsigned int from_id:4;
  unsigned int msg_type:3;
  unsigned int offset:11;
} msg_bit_info;

typedef union {
  unsigned int i;
  msg_packed_int b;
  msg_bit_info values;
} msg_packed;

msg_packed rxmsg_id,txmsg_id;

// unpack the vars from the payload of a MSG_REQ packet
typedef struct msg_req_data_packed_int {
  unsigned char b2;
  unsigned char b1;
  unsigned char b0;
} msg_req_data_packed_int;

typedef struct msq_req_data_bit_info {
  unsigned int varbyt:4;
  unsigned int spare:1;
  unsigned int varoffset:11;
  unsigned int varblk:4;
} msg_req_data_bit_info;

typedef union {
  msg_req_data_packed_int bytes;
  msg_req_data_bit_info values;
} msg_req_data_raw;

msg_req_data_raw msg_req_data;

HardwareSerial GPS(1);
UBXMessage ubxMessage;

CanFrame rxmsg, txmsg;

// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}
void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize)
{
  uint8_t byteread, index;

  Serial.print(F("GPSSend  "));

  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    if (byteread < 0x10)
    {
      Serial.print(F("0"));
    }
    Serial.print(byteread, HEX);
    Serial.print(F(" "));
  }

  Serial.println();
  Progmem_ptr = Progmem_ptr - arraysize;                  //set Progmem_ptr back to start

  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    GPS.write(byteread);
  }
  delay(100);
}


void loop() {
  Serial.print("GPGGAOff ");
  GPS_SendConfig(GPGGAOff, 16);

  Serial.print("GPGLLOff ");
  GPS_SendConfig(GPGLLOff, 16);

  Serial.print("GPGSVOff ");
  GPS_SendConfig(GPGSVOff, 16);

  Serial.print("GPVTGOff ");
  GPS_SendConfig(GPVTGOff, 16);

  Serial.print("GPGSAOff ");
  GPS_SendConfig(GPGSAOff, 16);

  Serial.print("GPRMCOff ");
  GPS_SendConfig(GPRMCOff, 16);


  Serial.print("NavPvtOff ");
  GPS_SendConfig(NavPvtOff, 16);

  Serial.print("NavPOSLLHOff ");
  GPS_SendConfig(NavPOSLLHOff, 16);

  Serial.print("NavSTATUSOff ");
  GPS_SendConfig(NavSTATUSOff, 16);
/*
  Serial.print("NavPOSLLHOn ");
  GPS_SendConfig(NavPOSLLHOn, 16);

  Serial.print("NavSTATUSOn ");
  GPS_SendConfig(NavSTATUSOn, 16);
*/
  Serial.print("NavPvtOn ");
  GPS_SendConfig(NavPvtOn, 16);

  Serial.print("Navrate10hz ");
  GPS_SendConfig(Navrate10hz, 14);

  Serial.println();
  Serial.println();
  Serial.flush();

  do {
    int msgType = processGPS();
    parseGPS(msgType);
    parseCAN();
  } while (1);
}

void parseCAN() {
  if (ESP32Can.readFrame(rxmsg, CANtimeout)) {
    if (DEBUG) {
      Serial.print(" Got CAN Frame for: "); Serial.print(rxmsg_id.values.to_id);
    }
    switch (rxmsg.identifier) {
      default:
      if (rxmsg.extd) { //assume this is a normal Megasquirt CAN protocol packet and decode the header
        rxmsg_id.i = rxmsg.identifier;
        if (rxmsg_id.values.to_id == myCANid) { // is this being sent to us?
          switch (rxmsg_id.values.msg_type) {
          case 1: // MSG_REQ - request data
            if (DEBUG) Serial.print(" Got CAN Req ");
            // the data required for the MSG_RSP header is packed into the first 3 data bytes
            msg_req_data.bytes.b0 = rxmsg.data[0];
            msg_req_data.bytes.b1 = rxmsg.data[1];
            msg_req_data.bytes.b2 = rxmsg.data[2];
            // Create the tx packet header
            txmsg_id.values.msg_type = 2; // MSG_RSP
            txmsg_id.values.to_id = msCANid;
            txmsg_id.values.from_id = myCANid;
            txmsg_id.values.block = msg_req_data.values.varblk;
            txmsg_id.values.offset = msg_req_data.values.varoffset;
            txmsg.extd = 1;
            txmsg.identifier = txmsg_id.i;
            txmsg. data_length_code = 8;
            // Use the same block and offset as JBPerf IO expander board for compatibility reasons
            // Docs at http://www.jbperf.com/io_extender/firmware/0_1_2/io_extender.ini (or latest version)
            if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 110 && validTime ) { // realtime clock
              if (DEBUG) Serial.print(" Got Time Req ");
              txmsg.data[0] = second;
              txmsg.data[1] = minute;
              txmsg.data[2] = hour;
              txmsg.data[3] = 0;
              txmsg.data[4] = day;
              txmsg.data[5] = month;
              txmsg.data[6] = year / 256;
              txmsg.data[7] = year % 256;
              // send the message!
              ESP32Can.writeFrame(txmsg);
            } else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 128 && gpsFixOk) { // gps1
              if (DEBUG) Serial.print("Got GPS1 Req");
              txmsg.data[0] = latdeg;
              txmsg.data[1] = latmin;
              txmsg.data[2] = latmmin / 256;
              txmsg.data[3] = latmmin % 256;
              txmsg.data[4] = londeg;
              txmsg.data[5] = lonmin;
              txmsg.data[6] = lonmmin / 256;
              txmsg.data[7] = lonmmin % 256;
              // send the message!
              ESP32Can.writeFrame(txmsg);           
            } else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 136) { // gps2
              if (DEBUG) Serial.print(" Got GPS2 Req ");
              txmsg.data[0] = lon < 0 ? 1 : 0; txmsg.data[0] += gpsFixOk ? 2 : 0;
              txmsg.data[1] = altk;
              txmsg.data[2] = altm  / 256;
              txmsg.data[3] = altm  % 256;
              txmsg.data[4] = gSpeed / 256;
              txmsg.data[5] = gSpeed % 256; 
              txmsg.data[6] = heading / 256;
              txmsg.data[7] = heading % 256;
              // send the message!
              ESP32Can.writeFrame(txmsg);
            } else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 2) { // ADC 1-4 - accelerometer
              if (DEBUG) Serial.print(" Got Accel Req ");
/*              sensors_event_t event;
              accel.getEvent(&event);
              // normalize +/- 4G to a 12 bit unsigned int value
              accel_x = (event.acceleration.x / 9.8 * 1023) + 2047;
              accel_y = (event.acceleration.y / 9.8 * 1023) + 2047;
              accel_z = (event.acceleration.z / 9.8 * 1023) + 2047;
              txmsg.data[0] = accel_x / 256;
              txmsg.data[1] = accel_x % 256;
              txmsg.data[2] = accel_y / 256;
              txmsg.data[3] = accel_y % 256;
              txmsg.data[4] = accel_z / 256;
              txmsg.data[5] = accel_z % 256;
              txmsg.data[6] = 0;
              txmsg.data[7] = 0;
              // send the message!
              ESP32Can.writeFrame(txmsg);
*/
            }
            break;
          }
        }
        if (DEBUG) Serial.println();
      } else {
        Serial.write("ID: ");
        Serial.print(rxmsg.identifier);
      }
    }
  }
}
// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}

void parseGPS(int msgType) {
  if (msgType == MT_NAV_POSLLH) {
    lat = ubxMessage.navPosllh.lat;
    lon = ubxMessage.navPosllh.lon;
    if (DEBUG) {
      Serial.print("iTOW:");
      Serial.print(ubxMessage.navPosllh.iTOW);
      Serial.print(" lat/lon: ");
      Serial.print(ubxMessage.navPosllh.lat / 10000000.0f);
      Serial.print(",");
      Serial.print(ubxMessage.navPosllh.lon / 10000000.0f);
      Serial.println();
    }
  } else if (msgType == MT_NAV_STATUS) {
    if (DEBUG) {
      Serial.print("gpsFix:");
      Serial.print(ubxMessage.navStatus.gpsFix);
      Serial.println();
    }
  } else if (msgType == MT_NAV_PVT) {
    validTime = ubxMessage.navPvt.valid >> 2 & 1;
    gpsFixOk = ubxMessage.navPvt.flags > 0;
    iTOW = ubxMessage.navPvt.iTOW;
    numSV = ubxMessage.navPvt.numSV;
    year = ubxMessage.navPvt.year;
    month = ubxMessage.navPvt.month;
    day = ubxMessage.navPvt.day;
    hour = ubxMessage.navPvt.hour;
    minute = ubxMessage.navPvt.minute;
    second = ubxMessage.navPvt.second;
    lat = ubxMessage.navPvt.lat;
    lon = ubxMessage.navPvt.lon;
    heading = ubxMessage.navPvt.heading;

    altk = round(ubxMessage.navPvt.hMSL / 1000000);
    altm = round(ubxMessage.navPvt.hMSL / 100); // mm to cm

    gSpeed = round(ubxMessage.navPvt.gSpeed / 100); // mm/s to cm/s

    unsigned int remainder;
    londeg = abs(lon / 10000000);
    remainder = abs(lon % 10000000);
    lonmin = round((remainder * 60) / 10000000);
    lonmmin = round(((remainder * 60) - lonmin * 10000000) / 1000);
    
    latdeg = lat / 10000000;
    remainder = abs(lat % 10000000);
    latmin = round((remainder * 60) / 10000000);
    latmmin = round(((remainder * 60) - latmin * 10000000) / 1000);
/*
    if (validTime) {
      rtc.setTime(epoch, us);
      Serial.print("System time set to: ");
      Serial.println(rtc.getDateTime(true));
    }
*/
    if (DEBUG) {
      Serial.print(" #SV: ");      Serial.print(numSV);
      Serial.print(" iTOW: ");      Serial.print(iTOW);
      Serial.print(" validTime: ");      Serial.print(validTime);
      Serial.print(" gpsFixOk: ");      Serial.print(int(gpsFixOk));
      Serial.print(" Date:");     Serial.print(year); Serial.print("/"); Serial.print(month); Serial.print("/"); Serial.print(day); Serial.print(" ");
        Serial.print(hour); Serial.print(":"); Serial.print(minute); Serial.print(":"); Serial.print(second);
      Serial.print(" lat/lon: "); Serial.print(lat); Serial.print(","); Serial.println(lon);
      Serial.print(" lat/lon: "); Serial.print(latdeg); Serial.print("."); Serial.print(latmin);
        Serial.print(","); Serial.print(londeg); Serial.print("."); Serial.println(lonmin);
      Serial.print(" gSpeed: ");  Serial.print(gSpeed / 256); Serial.print(gSpeed % 256); 
      Serial.println();
      Serial.print(" hMSL/altk/altm: "); Serial.print(ubxMessage.navPvt.hMSL); Serial.print(","); Serial.print(altk); Serial.print(","); Serial.println(altm);

    }
  }
}

// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];

  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while (GPS.available()) {
    byte c = GPS.read();

    if (fpos < 2) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if (c == UBX_HEADER[fpos])
        fpos++;
      else
        fpos = 0;  // Reset to beginning state.
    } else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.

      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ((fpos - 2) < payloadSize)
        ((unsigned char*)(&ubxMessage))[fpos - 2] = c;
      fpos++;
      if (fpos == 4) {
        // We have just received the second byte of the message type header,
        // so now we can check to see what kind of message it is.
        if (compareMsgHeader(NAV_POSLLH_HEADER)) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(NAV_POSLLH);
        } else if (compareMsgHeader(NAV_STATUS_HEADER)) {
          currentMsgType = MT_NAV_STATUS;
          payloadSize = sizeof(NAV_STATUS);
        } else if (compareMsgHeader(NAV_PVT_HEADER)) {
          currentMsgType = MT_NAV_PVT;
          payloadSize = sizeof(NAV_PVT);
        } else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }
      if (fpos == (payloadSize + 2)) {
        // All payload bytes have now been received, so we can calculate the
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
      } else if (fpos == (payloadSize + 3)) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if (c != checksum[0]) {
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0;
        }
      } else if (fpos == (payloadSize + 4)) { 
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0;  // We will reset the state regardless of whether the checksum matches.
        if (c == checksum[1]) {
          // Checksum matches, we have a valid message.
          return currentMsgType;
        }
      } else if (fpos > (payloadSize + 4)) {
        // We have now read more bytes than both the expected payload and checksum
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  return MT_NONE;
}
void setup() {
  // init variables
  validTime = gpsFixOk = false; 
  lat = lon = 0;
  gSpeed = -1;
  Serial.begin(115200);
  Serial.println("90_UBlox_GPS_Configuration Starting");
  Serial.println();
  GPS.begin(115200, SERIAL_8N1, 16, 17);  // GPS setup

  if(ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10)) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }
}