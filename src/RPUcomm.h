/*
 *  RPUComm.h
 *  Derived from PUcomm.h
 *  
 *  This file declares an Arduino library (C++ class) that implements the communication
 *  between the RATCHuTS and the RPU. The class inherits its protocol from the SerialComm
 *  class.
 */

#ifndef RPUComm_H
#define RPUComm_H

#include "SerialComm.h"

enum RPUMessages_t : uint8_t {
    RPU_NO_MESSAGE = 0,        // —
    RPU_SEND_STATUS,           // RATCHUTS→RPU
    RPU_SEND_RECORDS,          // RATCHUTS→RPU
    RPU_RESET,                 // RATCHUTS→RPU
    RPU_GO_MEASURE,            // RATCHUTS→RPU | duration(int32_t s), rate(int32_t s), opc(int8_t), tdlas(int8_t), tsen(int8_t)
    RPU_GO_STANDBY,            // RATCHUTS→RPU
    RPU_SET_BATT_T,            // RATCHUTS→RPU | setpoint(float °C)
    RPU_SET_V_LOW_BATT,        // RATCHUTS→RPU | threshold(float V)
    RPU_SET_V_CRIT_BATT,       // RATCHUTS→RPU | threshold(float V)
    RPU_SET_STATUS_RATE,       // RATCHUTS→RPU | interval(uint32_t s)
    RPU_PROFILE_RECORD,        // RATCHUTS→RPU
    RPU_NO_MORE_RECORDS,       // RPU→RATCHUTS
    RPU_STATUS,                // RPU→RATCHUTS | JSON(string)
    RPU_ERROR                  // RPU→RATCHUTS | message(string)
};


class RPUComm : public SerialComm {
public:
    RPUComm(Stream * serial_port);
    ~RPUComm() { };

    // RATCHuTS -> RPU (with params) -----------------------
    bool TX_GoMeasure(int32_t duration, int32_t rate, int8_t opc_power, int8_t tdlas_power, int8_t tsen_power);
    bool RX_GoMeasure(int32_t * duration, int32_t * rate, int8_t * opc_power, int8_t * tdlas_power, int8_t * tsen_power);


    // RPU -> RATCHuTS (with params) -----------------------

    bool TX_Status(uint32_t RPUTime, float VBattery, float ICharge, float Therm1T, float Therm2T, uint8_t HeaterStat);
    bool RX_Status(uint32_t * RPUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T, uint8_t * HeaterStat);

    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);
};

// ---------------------------------------------------------------------------
// RPU status packet bit-field widths
// Shared between RPU (encoder) and RATCHuTS (decoder).
// Packet version 1 — 320 bits = 40 bytes, big-endian (max packet size 250 bytes)
// ---------------------------------------------------------------------------
constexpr uint8_t  RPU_PKT_VERSION      = 1;
constexpr uint8_t  RPU_PKT_VER_BITS     = 4;   // packet format version
constexpr uint8_t  RPU_PKT_ID_BITS      = 16;  // board ID
constexpr uint8_t  RPU_PKT_STATE_BITS   = 4;   // RPUState enum
constexpr uint8_t  RPU_PKT_WDT_BITS     = 8;   // watchdog reset count
constexpr uint8_t  RPU_PKT_VIN_BITS     = 8;   // V_IN   × 0.1 V  (0–25.5 V)
constexpr uint8_t  RPU_PKT_V5_BITS      = 8;   // v_5V   × 0.1 V  (0–25.5 V)
constexpr uint8_t  RPU_PKT_BATV_BITS    = 8;   // bat_v  × 0.1 V  (0–25.5 V)
constexpr uint8_t  RPU_PKT_DUTY_BITS    = 7;   // heater duty      (0–100 %)
constexpr uint8_t  RPU_PKT_CHGI_BITS    = 7;   // chg_i  × 0.05 A (0–6.35 A)
constexpr uint8_t  RPU_PKT_TEMP_BITS    = 9;   // (T + 100) × 2   (-100 to +155 °C, 0.5 °C res)
constexpr uint8_t  RPU_PKT_CURR_BITS    = 12;  // subsystem I, mA  (0–4095 mA)
constexpr uint8_t  RPU_PKT_LAT_BITS     = 21;  // (lat + 90)  × 10000  (0–1 800 000)
constexpr uint8_t  RPU_PKT_LON_BITS     = 22;  // (lon + 180) × 10000  (0–3 600 000)
constexpr uint8_t  RPU_PKT_ALT_BITS     = 16;  // altitude, m (0–65 535)
constexpr uint8_t  RPU_PKT_SATS_BITS    = 5;   // satellite count (0–31)
constexpr uint8_t  RPU_PKT_GPS_DATE_BITS = 19; // GPS date, DDMMYY (Year is 20YY) — same encoding as ECUComm
constexpr uint8_t  RPU_PKT_GPS_TIME_BITS = 25; // GPS time, HHMMSSCC (seconds in 100ths) — same encoding as ECUComm
constexpr size_t   RPU_PKT_FW_VER_LEN   = 8;   // firmware-version string, fixed-length, NUL-padded
constexpr size_t   RPU_PKT_FW_VER_BITS  = RPU_PKT_FW_VER_LEN * 8;
constexpr size_t   RPU_PKT_BYTES        = 40;  // ceil(320 / 8); must be <= 250

// ---------------------------------------------------------------------------
// RPUPacket
// Holds the field values of an RPU status packet in engineering units and
// converts to/from the bit-packed 26-byte wire format. The scale/offset/
// range of each field is centralised here so the encoder (RPU) and decoder
// (RATCHuTS) stay in sync.
// ---------------------------------------------------------------------------
class RPUPacket {
public:
    RPUPacket() = default;

    // Setters (engineering units -> packed encoding) ---------
    void setBoardId(uint16_t id);
    void setState(uint8_t state);
    void setWdtCount(uint8_t count);
    void setVin(float volts);
    void setV5V(float volts);
    void setBatV(float volts);
    void setHeaterDuty(uint8_t percent);
    void setChgI(float amps);
    void setBatT(float celsius);
    void setPcbT(float celsius);
    void setPumpI(float milliamps);
    void setOpcI(float milliamps);
    void setTsenI(float milliamps);
    void setTdlasI(float milliamps);
    void setHeaterI(float milliamps);
    void setLat(double degrees);
    void setLon(double degrees);
    void setAlt(float meters);
    void setSats(uint8_t count);
    void setGpsDate(uint32_t date);   // DDMMYY, as returned by TinyGPSDate::value()
    void setGpsTime(uint32_t time);   // HHMMSSCC, as returned by TinyGPSTime::value()
    void setVer(const char* ver);

    // Getters (packed encoding -> engineering units) ---------
    uint16_t getBoardId()    const { return board_id_; }
    uint8_t  getState()      const { return state_; }
    uint8_t  getWdtCount()   const { return wdt_count_; }
    float    getVin()        const { return vin_raw_ / 10.0f; }
    float    getV5V()        const { return v5v_raw_ / 10.0f; }
    float    getBatV()       const { return bat_v_raw_ / 10.0f; }
    uint8_t  getHeaterDuty() const { return heater_duty_; }
    float    getChgI()       const { return chg_i_raw_ / 20.0f; }
    float    getBatT()       const { return (bat_t_raw_ / 2.0f) - 100.0f; }
    float    getPcbT()       const { return (pcb_t_raw_ / 2.0f) - 100.0f; }
    float    getPumpI()      const { return (float)pump_i_raw_; }
    float    getOpcI()       const { return (float)opc_i_raw_; }
    float    getTsenI()      const { return (float)tsen_i_raw_; }
    float    getTdlasI()     const { return (float)tdlas_i_raw_; }
    float    getHeaterI()    const { return (float)heater_i_raw_; }
    double   getLat()        const { return (lat_raw_ / 10000.0) - 90.0; }
    double   getLon()        const { return (lon_raw_ / 10000.0) - 180.0; }
    float    getAlt()        const { return (float)alt_raw_; }
    uint8_t  getSats()       const { return sats_; }
    uint32_t getGpsDate()    const { return gps_date_; }
    uint32_t getGpsTime()    const { return gps_time_; }
    const char* getVer()     const { return ver_; }

    // Byte-level pack / unpack using the bit-field widths above
    bool encode(uint8_t* buf, size_t buf_size) const;
    bool decode(const uint8_t* buf, size_t buf_size);

    // JSON serialisation in engineering units.
    String toJSON() const;

private:
    uint16_t board_id_     = 0;
    uint8_t  state_        = 0;
    uint8_t  wdt_count_    = 0;
    uint8_t  vin_raw_      = 0;   // x0.1 V
    uint8_t  v5v_raw_      = 0;   // x0.1 V
    uint8_t  bat_v_raw_    = 0;   // x0.1 V
    uint8_t  heater_duty_  = 0;   // %
    uint8_t  chg_i_raw_    = 0;   // x0.05 A
    uint16_t bat_t_raw_    = 0;   // (T + 100) x2
    uint16_t pcb_t_raw_    = 0;   // (T + 100) x2
    uint16_t pump_i_raw_   = 0;   // mA
    uint16_t opc_i_raw_    = 0;   // mA
    uint16_t tsen_i_raw_   = 0;   // mA
    uint16_t tdlas_i_raw_  = 0;   // mA
    uint16_t heater_i_raw_ = 0;   // mA
    uint32_t lat_raw_      = 0;   // (lat + 90)  x10000
    uint32_t lon_raw_      = 0;   // (lon + 180) x10000
    uint16_t alt_raw_      = 0;   // m
    uint8_t  sats_         = 0;
    uint32_t gps_date_     = 0;   // DDMMYY
    uint32_t gps_time_     = 0;   // HHMMSSCC
    char     ver_[RPU_PKT_FW_VER_LEN] = {0}; // NUL-padded firmware-version string
};

#endif /* RPUComm_H */
