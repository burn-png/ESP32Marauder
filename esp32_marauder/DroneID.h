/*
  DroneID.h
  Open Drone ID (ASTM F3411) BLE advertisement decoder.
  Ported from nyanBOX's drone_detector.cpp (BLE Basic ID + Location
  messages only -- the WiFi NAN/beacon path was not ported).
  https://github.com/jbohack/nyanBOX
*/
#pragma once

#ifndef DroneID_h
#define DroneID_h

#include <Arduino.h>

#define ODID_MESSAGE_SIZE 25
#define ODID_ID_SIZE 20
#define ODID_STR_SIZE 23

enum ODID_messagetype {
  ODID_MESSAGETYPE_BASIC_ID = 0,
  ODID_MESSAGETYPE_LOCATION = 1,
  ODID_MESSAGETYPE_AUTH = 2,
  ODID_MESSAGETYPE_SELF_ID = 3,
  ODID_MESSAGETYPE_SYSTEM = 4,
  ODID_MESSAGETYPE_OPERATOR_ID = 5,
  ODID_MESSAGETYPE_PACKED = 0xF,
};

struct ODID_BasicID_data {
  uint8_t IDType;
  uint8_t UAType;
  char UASID[ODID_ID_SIZE + 1];
};

struct ODID_Location_data {
  double Latitude;
  double Longitude;
  float AltitudeGeo;
};

// Returns true and fills id_out/loc_out (whichever applies) if msgData
// (25 bytes, already past the AD header/AppCode) is a Basic ID or
// Location message. loc_valid/id_valid indicate which struct was filled.
bool droneid_decode(const uint8_t* msgData, ODID_BasicID_data* id_out, bool* id_valid,
                     ODID_Location_data* loc_out, bool* loc_valid);

// Checks a raw BLE AD structure (as returned by NimBLE's getPayload(), i.e.
// starting at the length byte) for an Open Drone ID Service Data field
// (AD type 0x16, UUID 0xFFFA, AppCode 0x0D) at the given offset. Returns
// true and decodes into id_out/loc_out on match.
bool droneid_parse_payload(const uint8_t* payload, size_t len,
                            ODID_BasicID_data* id_out, bool* id_valid,
                            ODID_Location_data* loc_out, bool* loc_valid);

// Checks a raw 802.11 management frame payload (as delivered by the WiFi
// promiscuous callback, i.e. starting at the frame control field) for an
// Open Drone ID NAN action frame or beacon vendor-specific IE, and decodes
// the first Basic ID or Location message found. is_nan reports which
// carrier matched (for display only).
bool droneid_parse_80211(const uint8_t* payload, int len,
                          ODID_BasicID_data* id_out, bool* id_valid,
                          ODID_Location_data* loc_out, bool* loc_valid,
                          bool* is_nan);

#endif
