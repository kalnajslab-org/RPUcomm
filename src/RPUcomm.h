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
    RPU_GO_MEASURE,            // RATCHUTS→RPU | duration(int32_t s), rate(int32_t s), bat_temp(float °C), opc(int8_t), tdlas(int8_t), tsen(int8_t), rs41(int8_t)
    RPU_GO_STANDBY,            // RATCHUTS→RPU | bat_temp(float °C)
    RPU_SET_BATT_T,            // RATCHUTS→RPU | setpoint(float °C)
    RPU_SET_V_LOW_BATT,        // RATCHUTS→RPU | threshold(float V)
    RPU_SET_V_CRIT_BATT,       // RATCHUTS→RPU | threshold(float V)
    RPU_SET_STATUS_RATE,       // RATCHUTS→RPU | interval(uint32_t s)
    RPU_SET_TIME,              // RATCHUTS→RPU | epoch(uint32_t s, Unix UTC)
    RPU_PROFILE_RECORD,        // RATCHUTS→RPU
    RPU_NO_MORE_RECORDS,       // RPU→RATCHUTS
    RPU_STATUS,                // RPU→RATCHUTS | JSON (binary payload, raw string bytes)
    RPU_ERROR,                 // RPU→RATCHUTS | message(string)
    RPU_REGEN_RS41             // RATCHUTS→RPU | no params
};


class RPUComm : public SerialComm {
public:
    RPUComm(Stream * serial_port);
    ~RPUComm() { };

    // RATCHuTS -> RPU (with params) -----------------------
    bool TX_GoMeasure(int32_t duration, int32_t rate, float bat_temp, int8_t opc_power, int8_t tdlas_power, int8_t tsen_power, int8_t rs41_power);
    bool RX_GoMeasure(int32_t * duration, int32_t * rate, float * bat_temp, int8_t * opc_power, int8_t * tdlas_power, int8_t * tsen_power, int8_t * rs41_power);

    bool TX_GoStandby(float bat_temp);
    bool RX_GoStandby(float * bat_temp);

    bool TX_SetStatusRate(uint32_t interval);
    bool RX_SetStatusRate(uint32_t * interval);

    bool TX_SetTime(uint32_t epoch);
    bool RX_SetTime(uint32_t * epoch);


    // RPU -> RATCHuTS (with params) -----------------------

    bool TX_Status(const char * json);
    bool RX_Status(char * json, uint16_t buffer_size);

    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);
};

// ---------------------------------------------------------------------------
// RPU status packet bit-field widths
// Shared between RPU (encoder) and RATCHuTS (decoder).
// Packet version 1 — 336 bits = 42 bytes, big-endian (max packet size 250 bytes)
// ---------------------------------------------------------------------------
constexpr uint8_t  RPU_PKT_VERSION      = 1;
constexpr uint8_t  RPU_PKT_VER_BITS     = 4;   // packet format version
constexpr uint8_t  RPU_PKT_ID_BITS      = 16;  // board ID
constexpr uint8_t  RPU_PKT_STATE_BITS   = 4;   // RPUState enum
constexpr uint8_t  RPU_PKT_WDT_BITS     = 8;   // watchdog reset count
constexpr uint8_t  RPU_PKT_BUFREC_BITS  = 16;  // records buffered awaiting offload (0–65535)
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
constexpr size_t   RPU_PKT_BYTES        = 42;  // ceil(336 / 8); must be <= 250

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
    void setBufferedRecords(uint16_t count);
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
    uint16_t getBufferedRecords() const { return buffered_records_; }
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
    uint16_t buffered_records_ = 0; // records buffered awaiting offload
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

// ---------------------------------------------------------------------------
// RPURecord bit-field widths
// Shared between RPU (encoder) and TMmonster (decoder).
// Record version 2 — 392 bits = 49 bytes, big-endian, no padding.
//
// Each RPURecord carries 28 "fast" fields (period = 1, present every record)
// plus one fixed-size 40-bit "slot" from a round-robin rotation of 6 "slow"
// field groups (period = 6 — each slow group is therefore sent roughly once
// every 6 records). An RPU report is a block header followed by a sequence
// of RPURecords. The
// round-robin index is itself one of the fast fields, so a decoder always
// knows which slow fields are valid in a given record's slot. The slot is a
// fixed RPU_REC_SLOT_BITS regardless of index, so the overall record length
// never varies.
// ---------------------------------------------------------------------------
constexpr uint8_t  RPU_REC_VERSION       = 2;
constexpr uint8_t  RPU_REC_VER_BITS      = 4;    // packet format version

// --- Fast fields (period = 1, present in every record) ---------------------
constexpr uint8_t  RPU_REC_RR_IDX_BITS          = 4;  // round-robin slot index (0–5)
constexpr uint8_t  RPU_REC_ELAPSED_BITS         = 16; // elapsed seconds since GPSStartTime (0–65535 s)

constexpr uint8_t  RPU_REC_ALT_BITS             = 16; // altitude, m, raw
constexpr uint8_t  RPU_REC_GPS_DELTA_BITS       = 16; // (lat|lon - start) x50000, signed
constexpr uint8_t  RPU_REC_SATS_BITS            = 4;  // satellite count (0–15)
constexpr uint8_t  RPU_REC_GPS_AGE_BITS         = 4;  // GPS fix age, s, clamped (0–15 s)

constexpr uint8_t  RPU_REC_OPC_BITS             = 16; // OPC bin counts, raw
constexpr uint8_t  RPU_REC_TSEN_BITS            = 16; // TSEN raw counts (airt: 0–4095; pres/ptemp: top 16 bits of 24-bit count)

constexpr uint8_t  RPU_REC_RS41_T_BITS          = 16; // (T + 100) x436.9067  (-100 to +50 °C)
constexpr uint8_t  RPU_REC_RS41_P_BITS          = 16; // (ln(P) - 3.9120) x21525.87  (50–1050 hPa)
constexpr uint8_t  RPU_REC_RS41_RH_BITS         = 16; // (RH + 20) x543.1333  (-20 to +100 %RH)

constexpr uint8_t  RPU_REC_TDLAS_MIXING_RATIO_BITS = 18; // TDLAS mixing ratio x100 (0–2621.43)
constexpr uint8_t  RPU_REC_TDLAS_BACKGROUND_BITS   = 12; // TDLAS background, raw counts (0–4095)
constexpr uint8_t  RPU_REC_TDLAS_PEAK_BITS         = 9;  // TDLAS peak x10 (0–51.1)
constexpr uint8_t  RPU_REC_TDLAS_RATIO_BITS        = 5;  // TDLAS ratio x10 (0–3.1)
constexpr uint8_t  RPU_REC_TDLAS_LASER_TEMP_BITS   = 12; // TDLAS laser temp x100 (0–40.95 °C)
constexpr uint8_t  RPU_REC_TDLAS_MR_MAX_RATIO_BITS = 7;  // TDLAS max mixing ratio x10 (0–12.7)
constexpr uint8_t  RPU_REC_TDLAS_STATUS_BITS       = 5;  // TDLAS instrument status code (0–31)
constexpr uint8_t  RPU_REC_TDLAS_CLUSTER_IDX_BITS  = 4;  // TDLAS cluster index (0–15)
constexpr uint8_t  RPU_REC_TDLAS_CLUSTER_BITS      = 14; // TDLAS cluster value x100 (0–163.83), per channel

// --- Round-robin slow fields (period = 6; one fixed-size slot per record) --
constexpr uint8_t  RPU_REC_HDG_BITS        = 8;  // RS41 heading, x256/360 (0–360°, ~1.41° res)
constexpr uint8_t  RPU_REC_BEMF_BITS       = 16; // pump BEMF, V x1000
constexpr uint8_t  RPU_REC_HKCURR_BITS     = 8;  // subsystem currents, mA/4 (0–1020 mA, 4 mA res)
constexpr uint8_t  RPU_REC_V5V_BITS        = 8;  // V x50  (0–5.10 V, 0.02 V res)
constexpr uint8_t  RPU_REC_HKTEMP_BITS     = 8;  // (T + 100), 1 °C res (-100 to 155 °C)
constexpr uint8_t  RPU_REC_VOLT_BITS       = 12; // battery voltage x100 (0–40.95 V)
constexpr uint8_t  RPU_REC_HEATER_BITS     = 4;  // heater status (bit0: battery heater on)
constexpr uint8_t  RPU_REC_RS41_STATUS_BITS = 8;  // RS41 status flags byte (8 flags packed as bits)
// Bit definitions for the RS41 status byte (matches RS41StatusFlags_t field order):
constexpr uint8_t  RPU_REC_RS41_HIGH_INTERNAL_TEMP  = (1u << 0); // S.2: high internal temperature
constexpr uint8_t  RPU_REC_RS41_REGEN_TEMP_LOW      = (1u << 1); // S.3: regen temperature low
constexpr uint8_t  RPU_REC_RS41_PTU_FAILURE         = (1u << 2); // S.4: PTU failure
constexpr uint8_t  RPU_REC_RS41_FLASH_FAILURE       = (1u << 3); // S.5: flash failure
constexpr uint8_t  RPU_REC_RS41_LOW_INPUT_VOLTAGE   = (1u << 4); // E.6: low input voltage
constexpr uint8_t  RPU_REC_RS41_NOT_CALIBRATED      = (1u << 5); // E.7: not calibrated
constexpr uint8_t  RPU_REC_RS41_NO_PRESSURE_MODULE  = (1u << 6); // E.8: no pressure module
constexpr uint8_t  RPU_REC_RS41_DISCONNECTED_BOOM   = (1u << 7); // E.9: disconnected boom
constexpr uint8_t  RPU_REC_SLOT_PAD_BITS   = 8;  // padding within the two-field 40-bit slots (indices 0-3)
constexpr size_t   RPU_REC_SLOT_BITS       = 40; // fixed round-robin slot size

constexpr size_t   RPU_RECORD_BYTES        = 49; // (352 fast + 40 slow) / 8
constexpr size_t   RPU_BLOCK_HDR_BYTES     = 12; // epoch_time (uint32) + gps_lat (int32) + gps_lon (int32)

// ---------------------------------------------------------------------------
// RPURecord
// Holds one tickMeasure() sample (GPS, OPC, TSEN, RS41, TDLAS, housekeeping)
// and converts to/from the bit-packed wire format (RPU_REC_VERSION 1).
//
// Fast fields (period = 1) are present in every record. Slow fields
// (period = 8) are set on every tick, but encode() only serialises the one
// 40-bit slot selected by setRoundRobinIdx(); callers are expected to cycle
// the round-robin index 0..7 across successive records so that all 22 slow
// fields are eventually transmitted.
// ---------------------------------------------------------------------------
class RPURecord {
public:
    RPURecord();

    // Resets the round-robin slot rotation to slot 0. Call once per MEASURE session.
    void resetRotation();
    // Advances the round-robin slot rotation to the next slot (mod 8). Call once per record,
    // after encode()/push() so the slot just encoded is preserved for that record.
    void advanceRotation();

    // Setters (engineering units -> packed encoding) ---------

    // Fast fields (period = 1)
    void setElapsedS(uint32_t seconds);
    void setAlt(float meters);
    void setLatDelta(double degrees);
    void setLonDelta(double degrees);
    void setSats(uint8_t count);
    void setGpsAge(uint32_t seconds);
    void setOpcD300(uint16_t count);
    void setOpcD2000(uint16_t count);
    void setTsenAirt(uint16_t raw);
    void setTsenPres(uint32_t raw);
    void setTsenPtemp(uint32_t raw);
    void setRs41AirT(float celsius);
    void setRs41Pres(float millibar);
    void setRs41Humidity(float percent);
    void setRs41HSensorT(float celsius);
    void setTdlasMixingRatio(float value); // mixing ratio x100 (0–2621.43)
    void setTdlasBackground(float value);  // background, raw counts (0–4095)
    void setTdlasPeak(float value);        // peak x10 (0–51.1)
    void setTdlasRatio(float value);       // ratio x10 (0–3.1)
    void setTdlasLaserTemp(float celsius); // laser temperature x100 (0–40.95°C)
    void setTdlasMrMaxRatio(float value);  // max mixing ratio x10 (0–12.7)
    void setTdlasStatus(uint8_t status);   // instrument status code (0–31)
    void setTdlasClusterIdx(uint8_t idx);  // cluster index (0–15)
    void setTdlasCluster1(float value);    // cluster value 1 x100 (0–163.83)
    void setTdlasCluster2(float value);    // cluster value 2 x100 (0–163.83)
    void setTdlasCluster3(float value);    // cluster value 3 x100 (0–163.83)
    void setTdlasCluster4(float value);    // cluster value 4 x100 (0–163.83)

    // Slow / round-robin fields (period = 6)
    void setOpcD500(uint16_t count);
    void setOpcD700(uint16_t count);
    void setOpcD1000(uint16_t count);
    void setOpcD3000(uint16_t count);
    void setOpcD5000(uint16_t count);
    void setOpcD2500(uint16_t count);   // spec "10000nm" slot; ROPCData has no 10000nm channel
    void setRs41Hdg(float degrees);
    void setBemfV(float volts);
    void setRs41Status(uint8_t flags);  // 8 flags packed as bits (see RS41StatusFlags_t)
    void setTsenI(float milliamps);
    void setOpcI(float milliamps);
    void setPumpI(float milliamps);
    void setTdlasI(float milliamps);
    void setV5V(float volts);
    void setBatT(float celsius);
    void setPumpT(float celsius);
    void setPcbT(float celsius);
    void setBatV(float volts);
    void setHeaterStat(uint8_t status);

    // Getters (packed encoding -> engineering units) ---------

    // Fast fields
    uint32_t getElapsedS()     const { return elapsed_s_; }
    float    getAlt()          const { return (float)alt_raw_; }
    double   getLatDelta()     const { return lat_delta_raw_ / 50000.0; }
    double   getLonDelta()     const { return lon_delta_raw_ / 50000.0; }
    uint8_t  getSats()          const { return sats_; }
    uint32_t getGpsAge()       const { return gps_age_s_; }
    uint16_t getOpcD300()      const { return opc_d300_; }
    uint16_t getOpcD2000()     const { return opc_d2000_; }
    uint16_t getTsenAirt()     const { return tsen_airt_raw_; }
    uint16_t getTsenPres()     const { return tsen_pres_raw_; }
    uint16_t getTsenPtemp()    const { return tsen_ptemp_raw_; }
    float    getRs41AirT()     const { return (rs41_air_t_raw_ / 436.9067f) - 100.0f; }
    float    getRs41Pres()     const { return expf((rs41_pres_raw_ / 21525.87f) + 3.9120f); }
    float    getRs41Humidity() const { return (rs41_humidity_raw_ / 543.1333f) - 20.0f; }
    float    getRs41HSensorT() const { return (rs41_hsensor_t_raw_ / 436.9067f) - 100.0f; }

    float    getTdlasMixingRatio() const { return tdlas_mixing_ratio_raw_ / 100.0f; }
    float    getTdlasBackground() const { return (float)tdlas_background_raw_; }
    float    getTdlasPeak()       const { return tdlas_peak_raw_ / 10.0f; }
    float    getTdlasRatio()      const { return tdlas_ratio_raw_ / 10.0f; }
    float    getTdlasLaserTemp()  const { return tdlas_laser_temp_raw_ / 100.0f; }
    float    getTdlasMrMaxRatio() const { return tdlas_mr_max_ratio_raw_ / 10.0f; }
    uint8_t  getTdlasStatus()     const { return tdlas_status_; }
    uint8_t  getTdlasClusterIdx() const { return tdlas_cluster_idx_; }
    float    getTdlasCluster1()   const { return tdlas_cluster_1_raw_ / 100.0f; }
    float    getTdlasCluster2()   const { return tdlas_cluster_2_raw_ / 100.0f; }
    float    getTdlasCluster3()   const { return tdlas_cluster_3_raw_ / 100.0f; }
    float    getTdlasCluster4()   const { return tdlas_cluster_4_raw_ / 100.0f; }

    // Slow / round-robin fields
    uint16_t getOpcD500()      const { return opc_d500_; }
    uint16_t getOpcD700()      const { return opc_d700_; }
    uint16_t getOpcD1000()     const { return opc_d1000_; }
    uint16_t getOpcD3000()     const { return opc_d3000_; }
    uint16_t getOpcD5000()     const { return opc_d5000_; }
    uint16_t getOpcD2500()     const { return opc_d2500_; }
    float    getRs41Hdg()      const { return rs41_hdg_raw_ * (360.0f / 256.0f); }
    float    getBemfV()        const { return bemf_v_raw_ / 1000.0f; }
    uint8_t  getRs41Status()   const { return rs41_status_; }
    float    getTsenI()        const { return tsen_i_raw_ * 4.0f; }
    float    getOpcI()         const { return opc_i_raw_ * 4.0f; }
    float    getPumpI()        const { return pump_i_raw_ * 4.0f; }
    float    getTdlasI()       const { return tdlas_i_raw_ * 4.0f; }
    float    getV5V()          const { return v5v_raw_ / 50.0f; }
    float    getBatT()         const { return (float)bat_t_raw_ - 100.0f; }
    float    getPumpT()        const { return (float)pump_t_raw_ - 100.0f; }
    float    getPcbT()         const { return (float)pcb_t_raw_ - 100.0f; }
    float    getBatV()         const { return bat_v_raw_ / 100.0f; }
    uint8_t  getHeaterStat()   const { return heater_stat_; }

    // Byte-level pack / unpack using the bit-field widths above
    bool encode(uint8_t* buf, size_t buf_size) const;
    bool decode(const uint8_t* buf, size_t buf_size);

    // Encodes a block header (RPU_BLOCK_HDR_BYTES) to prepend to a sequence
    // of encoded RPURecords. Contains epoch_time, gps_lat and gps_lon so the
    // decoder can reconstruct absolute time and position from the per-record deltas.
    bool encodeBlockHeader(uint8_t* buf, size_t buf_size) const;

    // Decodes a block header (RPU_BLOCK_HDR_BYTES) from the start of a received
    // binary block, populating epoch_time, gps_lat, and gps_lon.
    bool decodeBlockHeader(const uint8_t* buf, size_t buf_size);

    // JSON serialisation in engineering units.
    String toJSON() const;

    // GPS reference position and time — NOT included in the bit-packed record.
    // Set by the program populating the record; used as metadata (e.g. block-
    // transfer header) so receivers can reconstruct absolute position from the
    // compressed lat/lon deltas. Programs that only call decode() will get 0.
    void     setGpsLat(double degrees)  { gps_lat_  = (int32_t)(degrees * 1.0e6); }
    void     setGpsLon(double degrees)  { gps_lon_  = (int32_t)(degrees * 1.0e6); }
    double   getGpsLat() const          { return gps_lat_ / 1.0e6; }
    double   getGpsLon() const          { return gps_lon_ / 1.0e6; }
    void     setEpochTime(uint32_t t)   { epoch_time_ = t; }  // Unix epoch (UTC)
    uint32_t getEpochTime() const       { return epoch_time_; }

private:
    // Fast fields (period = 1)
    uint16_t elapsed_s_          = 0; // s since GPSStartTime
    uint16_t alt_raw_            = 0; // m
    int16_t  lat_delta_raw_      = 0; // (lat - GPSStartLat) x50000
    int16_t  lon_delta_raw_      = 0; // (lon - GPSStartLon) x50000
    uint8_t  sats_               = 0; // 0-15
    uint8_t  gps_age_s_          = 0; // 0-15 s
    uint16_t opc_d300_           = 0;
    uint16_t opc_d2000_          = 0;
    uint16_t tsen_airt_raw_      = 0; // raw 12-bit A/D count
    uint16_t tsen_pres_raw_      = 0; // top 16 bits of raw 24-bit count
    uint16_t tsen_ptemp_raw_     = 0; // top 16 bits of raw 24-bit count
    uint16_t rs41_air_t_raw_     = 0; // (T + 100) x436.9067, -100 to +50 °C
    uint16_t rs41_pres_raw_      = 0; // (ln(P) - 3.9120) x21525.87, 50-1050 hPa
    uint16_t rs41_humidity_raw_  = 0; // (RH + 20) x543.1333, -20 to +100 %RH
    uint16_t rs41_hsensor_t_raw_ = 0; // (T + 100) x436.9067, -100 to +50 °C
    uint32_t tdlas_mixing_ratio_raw_ = 0; // x100, 18 bits (0-2621.43)
    uint16_t tdlas_background_raw_  = 0; // raw counts, 12 bits (0-4095)
    uint16_t tdlas_peak_raw_        = 0; // x10, 9 bits (0-51.1)
    uint8_t  tdlas_ratio_raw_       = 0; // x10, 5 bits (0-3.1)
    uint16_t tdlas_laser_temp_raw_  = 0; // x100, 12 bits (0-40.95 °C)
    uint8_t  tdlas_mr_max_ratio_raw_ = 0; // x10, 7 bits (0-12.7)
    uint8_t  tdlas_status_          = 0; // 0-31, instrument status code
    uint8_t  tdlas_cluster_idx_     = 0; // 0-15, cluster index
    uint16_t tdlas_cluster_1_raw_   = 0; // x100, 14 bits (0-163.83)
    uint16_t tdlas_cluster_2_raw_   = 0; // x100, 14 bits (0-163.83)
    uint16_t tdlas_cluster_3_raw_   = 0; // x100, 14 bits (0-163.83)
    uint16_t tdlas_cluster_4_raw_   = 0; // x100, 14 bits (0-163.83)
    // Current round-robin slot (0-7), advanced via resetRotation()/advanceRotation().
    uint8_t  round_robin_idx_    = 0;

    // Slow / round-robin fields (period = 8)
    uint16_t opc_d500_           = 0;
    uint16_t opc_d700_           = 0;
    uint16_t opc_d1000_          = 0;
    uint16_t opc_d3000_          = 0;
    uint16_t opc_d5000_          = 0;
    uint16_t opc_d2500_          = 0; // spec "10000nm" slot
    uint8_t  rs41_hdg_raw_       = 0; // degrees * 256/360
    uint16_t bemf_v_raw_         = 0; // x1000 V
    uint8_t  rs41_status_        = 0; // 8 flags packed as bits
    uint8_t  tsen_i_raw_         = 0; // mA / 4
    uint8_t  opc_i_raw_          = 0; // mA / 4
    uint8_t  pump_i_raw_         = 0; // mA / 4
    uint8_t  tdlas_i_raw_        = 0; // mA / 4
    uint8_t  v5v_raw_            = 0; // V x50
    uint8_t  bat_t_raw_          = 0; // T + 100
    uint8_t  pump_t_raw_         = 0; // T + 100
    uint8_t  pcb_t_raw_          = 0; // T + 100
    uint16_t bat_v_raw_          = 0; // V x100, 12 bits
    uint8_t  heater_stat_        = 0; // 4 bits

    // Metadata — not bit-packed, not transmitted in the compressed record.
    uint32_t epoch_time_         = 0; // Unix epoch (UTC)
    int32_t  gps_lat_            = 0; // degrees * 1e6
    int32_t  gps_lon_            = 0; // degrees * 1e6
};

#endif /* RPUComm_H */
