/*
  DroneID.cpp
  Ported from nyanBOX's drone_detector.cpp (BLE path only).
  https://github.com/jbohack/nyanBOX
*/
#include "DroneID.h"

#define LATLON_MULT 10000000.0
#define ALT_DIV 0.5f
#define ALT_ADDER 1000.0f
#define INV_ALT 0xFFFF

static double decodeLatLon(int32_t LatLon_enc) {
  return (double)LatLon_enc / (double)LATLON_MULT;
}

static float decodeAltitude(uint16_t Alt_enc) {
  if (Alt_enc == 0xFFFF) return -1000.0f;
  return (float)Alt_enc * ALT_DIV - ALT_ADDER;
}

static int decodeBasicIDMessage(ODID_BasicID_data* outData, const uint8_t* inEncoded) {
  uint8_t msgType = (inEncoded[0] >> 4) & 0x0F;
  if (msgType != ODID_MESSAGETYPE_BASIC_ID) return -1;

  outData->IDType = (inEncoded[1] >> 4) & 0x0F;
  outData->UAType = inEncoded[1] & 0x0F;

  memcpy(outData->UASID, &inEncoded[2], ODID_ID_SIZE);
  outData->UASID[ODID_ID_SIZE] = '\0';

  // Strip anything non-printable so it's safe to display
  for (int i = 0; i < ODID_ID_SIZE; i++) {
    if (outData->UASID[i] < 32 || outData->UASID[i] > 126) {
      outData->UASID[i] = '\0';
      break;
    }
  }

  return 0;
}

static int decodeLocationMessage(ODID_Location_data* outData, const uint8_t* inEncoded) {
  uint8_t msgType = (inEncoded[0] >> 4) & 0x0F;
  if (msgType != ODID_MESSAGETYPE_LOCATION) return -1;

  int32_t lat = (int32_t)((uint32_t)inEncoded[5] |
                           ((uint32_t)inEncoded[6] << 8) |
                           ((uint32_t)inEncoded[7] << 16) |
                           ((uint32_t)inEncoded[8] << 24));
  outData->Latitude = decodeLatLon(lat);

  int32_t lon = (int32_t)((uint32_t)inEncoded[9] |
                           ((uint32_t)inEncoded[10] << 8) |
                           ((uint32_t)inEncoded[11] << 16) |
                           ((uint32_t)inEncoded[12] << 24));
  outData->Longitude = decodeLatLon(lon);

  uint16_t altGeo = (uint16_t)inEncoded[15] | ((uint16_t)inEncoded[16] << 8);
  outData->AltitudeGeo = decodeAltitude(altGeo);

  return 0;
}

bool droneid_decode(const uint8_t* msgData, ODID_BasicID_data* id_out, bool* id_valid,
                     ODID_Location_data* loc_out, bool* loc_valid) {
  *id_valid = false;
  *loc_valid = false;

  uint8_t msgType = (msgData[0] >> 4) & 0x0F;

  if (msgType == ODID_MESSAGETYPE_BASIC_ID) {
    if (decodeBasicIDMessage(id_out, msgData) == 0) {
      *id_valid = true;
      return true;
    }
  } else if (msgType == ODID_MESSAGETYPE_LOCATION) {
    if (decodeLocationMessage(loc_out, msgData) == 0) {
      *loc_valid = true;
      return true;
    }
  }

  return false;
}

bool droneid_parse_payload(const uint8_t* payload, size_t len,
                            ODID_BasicID_data* id_out, bool* id_valid,
                            ODID_Location_data* loc_out, bool* loc_valid) {
  // Scan the whole AD blob for a Service Data (0x16) field carrying the
  // ASTM Remote ID UUID (0xFFFA, little-endian FA FF) + AppCode 0x0D,
  // rather than assuming it's the very first AD structure.
  if (len < 7) return false;

  for (size_t i = 0; i + 6 < len; i++) {
    if (payload[i] == 0x16 && payload[i + 1] == 0xFA && payload[i + 2] == 0xFF &&
        payload[i + 3] == 0x0D) {
      const uint8_t* msg = &payload[i + 5];
      size_t remaining = len - (i + 5);
      if (remaining < ODID_MESSAGE_SIZE) return false;
      return droneid_decode(msg, id_out, id_valid, loc_out, loc_valid);
    }
  }

  return false;
}

// Wi-Fi NAN "Service Discovery" destination used by Open Drone ID
// (multicast 51:6f:9a:01:00:00), matched against the frame's dest addr
// (offset 4 in an 802.11 mgmt frame).
static const uint8_t nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};

bool droneid_parse_80211(const uint8_t* payload, int len,
                          ODID_BasicID_data* id_out, bool* id_valid,
                          ODID_Location_data* loc_out, bool* loc_valid,
                          bool* is_nan) {
  *id_valid = false;
  *loc_valid = false;
  *is_nan = false;

  if (len <= 40) return false;

  // NAN action frame: dest addr (bytes 4-9) is the ODID multicast address.
  // The message is packed somewhere after the NAN header; scan for a valid
  // Basic ID/Location message type byte like nyanBOX does.
  if (memcmp(nan_dest, &payload[4], 6) == 0) {
    for (int offset = 26; offset < len - ODID_MESSAGE_SIZE; offset++) {
      if (droneid_decode(&payload[offset], id_out, id_valid, loc_out, loc_valid)) {
        *is_nan = true;
        return true;
      }
    }
    return false;
  }

  // Beacon frame with a vendor-specific IE (tag 0xDD) carrying the ASD-STAN
  // (90:3a:e6) or ASTM (fa:0b:bc) OUI.
  if (payload[0] == 0x80) {
    int offset = 36;
    while (offset < len - 6) {
      uint8_t typ = payload[offset];
      uint8_t taglen = payload[offset + 1];

      if (typ == 0xdd && taglen >= 4 &&
          ((payload[offset + 2] == 0x90 && payload[offset + 3] == 0x3a && payload[offset + 4] == 0xe6) ||
           (payload[offset + 2] == 0xfa && payload[offset + 3] == 0x0b && payload[offset + 4] == 0xbc))) {
        int j = offset + 7;
        if (j < len - ODID_MESSAGE_SIZE) {
          return droneid_decode(&payload[j], id_out, id_valid, loc_out, loc_valid);
        }
      }
      offset += taglen + 2;
      if (offset >= len) break;
    }
  }

  return false;
}
