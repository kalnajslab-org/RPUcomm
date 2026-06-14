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
    RPU_PROFILE_RECORD,        // RATCHUTS→RPU
    RPU_NO_MORE_RECORDS,       // RPU→RATCHUTS
    RPU_STATUS,                // RPU→RATCHUTS | JSON (binary payload, raw string bytes)
    RPU_ERROR                  // RPU→RATCHUTS | message(string)
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
// RPU report bit-field widths
// Shared between RPU (encoder) and RATCHuTS (decoder).
// Packet version 1 — 304 bits = 38 bytes, big-endian, no padding.
//
// Matches the "Profiler TM Format 2026" spec: 5000 records/profile at 38
// bytes each (190,000 bytes, exactly the spec's target). Each record carries
// 20 "fast" fields (period = 1, present every record) plus one fixed-size
// 40-bit "slot" from a round-robin rotation of 22 "slow" fields (period = 8
// — each slow field is therefore updated roughly once every 8 records). The
// round-robin index is itself one of the fast fields, so a decoder always
// knows which slow fields are valid in a given record's slot. The slot is a
// fixed RPU_RPT_SLOT_BITS regardless of index, so the overall record length
// never varies.
// ---------------------------------------------------------------------------
constexpr uint8_t  RPU_RPT_VERSION       = 1;
constexpr uint8_t  RPU_RPT_VER_BITS      = 4;    // packet format version

// --- Fast fields (period = 1, present in every record) ---------------------
constexpr uint8_t  RPU_RPT_RR_IDX_BITS     = 4;  // round-robin slot index (0–7)
constexpr uint8_t  RPU_RPT_ELAPSED_BITS    = 16; // elapsed seconds since GPSStartTime (0–65535 s)
constexpr uint8_t  RPU_RPT_ALT_BITS        = 16; // altitude, m, raw
constexpr uint8_t  RPU_RPT_GPS_DELTA_BITS  = 16; // (lat|lon - start) x50000, signed
constexpr uint8_t  RPU_RPT_SATS_BITS       = 4;  // satellite count (0–15)
constexpr uint8_t  RPU_RPT_GPS_AGE_BITS    = 4;  // GPS fix age, s, clamped (0–15 s)
constexpr uint8_t  RPU_RPT_OPC_BITS        = 16; // OPC bin counts, raw
constexpr uint8_t  RPU_RPT_TSEN_BITS       = 16; // TSEN raw counts (airt: 0–4095; pres/ptemp: top 16 bits of 24-bit count)
constexpr uint8_t  RPU_RPT_RS41_T_BITS     = 16; // (T + 100) x100  (-100.00 to 555.35 °C)
constexpr uint8_t  RPU_RPT_RS41_P_BITS     = 16; // pressure x10    (0–6553.5 mb)
constexpr uint8_t  RPU_RPT_RS41_RH_BITS    = 16; // RH x100         (0–655.35 %)
constexpr uint8_t  RPU_RPT_TDLAS_VMR_BITS  = 10; // TDLAS VMR_ave x10, provisional (0–102.3)
constexpr uint8_t  RPU_RPT_TDLAS_BKG_BITS  = 12; // TDLAS bkg x100, provisional (0–40.95)
constexpr uint8_t  RPU_RPT_TDLAS_PEAK_BITS = 8;  // TDLAS peak x10, provisional (0–25.5)
constexpr uint8_t  RPU_RPT_TDLAS_RATIO_BITS= 10; // TDLAS ratio x1000, provisional (0–1.023)

// --- Round-robin slow fields (period = 8; one fixed-size slot per record) --
constexpr uint8_t  RPU_RPT_MAGXY_BITS      = 16; // RS41 magnetometer X-Y, counts + 1000
constexpr uint8_t  RPU_RPT_BEMF_BITS       = 16; // pump BEMF, V x1000
constexpr uint8_t  RPU_RPT_SPEC_BITS       = 16; // TDLAS spectra, raw passthrough, provisional
constexpr uint8_t  RPU_RPT_HKCURR_BITS     = 8;  // subsystem currents, mA/4 (0–1020 mA, 4 mA res)
constexpr uint8_t  RPU_RPT_V5V_BITS        = 8;  // V x50  (0–5.10 V, 0.02 V res)
constexpr uint8_t  RPU_RPT_HKTEMP_BITS     = 8;  // (T + 100), 1 °C res (-100 to 155 °C)
constexpr uint8_t  RPU_RPT_VOLT_BITS       = 12; // battery voltage x100 (0–40.95 V)
constexpr uint8_t  RPU_RPT_HEATER_BITS     = 4;  // heater status (bit0: battery heater on)
constexpr uint8_t  RPU_RPT_SLOT_PAD_BITS   = 8;  // padding within the two-field 40-bit slots (indices 0-5)
constexpr size_t   RPU_RPT_SLOT_BITS       = 40; // fixed round-robin slot size

constexpr size_t   RPU_RECORD_BYTES        = 38; // (4 + 260 + 40) / 8, no padding needed

// ---------------------------------------------------------------------------
// RPURecord
// Holds one tickMeasure() sample (GPS, OPC, TSEN, RS41, TDLAS, housekeeping)
// and converts to/from the bit-packed wire format (RPU_RPT_VERSION 1).
//
// Fast fields (period = 1) are present in every record. Slow fields
// (period = 8) are set on every tick, but encode() only serialises the one
// 40-bit slot selected by setRoundRobinIdx(); callers are expected to cycle
// the round-robin index 0..7 across successive records so that all 22 slow
// fields are eventually transmitted.
// ---------------------------------------------------------------------------
class RPURecord {
public:
    // Captures the current round-robin slot (see resetRotation()/advanceRotation()).
    RPURecord();

    // Resets the round-robin slot rotation to slot 0. Call once per MEASURE session.
    static void resetRotation();

    // Advances the round-robin slot rotation to the next slot (mod 8). Call once per record.
    static void advanceRotation();

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
    void setTdlasMrAvg(float value);
    void setTdlasBkg(float value);
    void setTdlasPeak(float value);
    void setTdlasRatio(float value);

    // Slow / round-robin fields (period = 8)
    void setOpcD500(uint16_t count);
    void setOpcD700(uint16_t count);
    void setOpcD1000(uint16_t count);
    void setOpcD3000(uint16_t count);
    void setOpcD5000(uint16_t count);
    void setOpcD2500(uint16_t count);   // spec "10000nm" slot; ROPCData has no 10000nm channel
    void setRs41MagXY(int32_t counts);
    void setBemfV(float volts);
    void setTdlasSpec1(float value);
    void setTdlasSpec2(float value);
    void setTdlasSpec3(float value);
    void setTdlasSpec4(float value);
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
    float    getRs41AirT()     const { return (rs41_air_t_raw_ / 100.0f) - 100.0f; }
    float    getRs41Pres()     const { return rs41_pres_raw_ / 10.0f; }
    float    getRs41Humidity() const { return rs41_humidity_raw_ / 100.0f; }
    float    getRs41HSensorT() const { return (rs41_hsensor_t_raw_ / 100.0f) - 100.0f; }
    float    getTdlasMrAvg()   const { return tdlas_mr_avg_raw_ / 10.0f; }
    float    getTdlasBkg()     const { return tdlas_bkg_raw_ / 100.0f; }
    float    getTdlasPeak()    const { return tdlas_peak_raw_ / 10.0f; }
    float    getTdlasRatio()   const { return tdlas_ratio_raw_ / 1000.0f; }

    // Slow / round-robin fields
    uint16_t getOpcD500()      const { return opc_d500_; }
    uint16_t getOpcD700()      const { return opc_d700_; }
    uint16_t getOpcD1000()     const { return opc_d1000_; }
    uint16_t getOpcD3000()     const { return opc_d3000_; }
    uint16_t getOpcD5000()     const { return opc_d5000_; }
    uint16_t getOpcD2500()     const { return opc_d2500_; }
    int32_t  getRs41MagXY()    const { return (int32_t)rs41_mag_xy_raw_ - 1000; }
    float    getBemfV()        const { return bemf_v_raw_ / 1000.0f; }
    float    getTdlasSpec1()   const { return (float)tdlas_spec_1_raw_; }
    float    getTdlasSpec2()   const { return (float)tdlas_spec_2_raw_; }
    float    getTdlasSpec3()   const { return (float)tdlas_spec_3_raw_; }
    float    getTdlasSpec4()   const { return (float)tdlas_spec_4_raw_; }
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

    // JSON serialisation in engineering units.
    String toJSON() const;

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
    uint16_t rs41_air_t_raw_     = 0; // (T + 100) x100
    uint16_t rs41_pres_raw_      = 0; // x10 mb
    uint16_t rs41_humidity_raw_  = 0; // x100 %
    uint16_t rs41_hsensor_t_raw_ = 0; // (T + 100) x100
    uint16_t tdlas_mr_avg_raw_   = 0; // x10, provisional (0-102.3)
    uint16_t tdlas_bkg_raw_      = 0; // x100, provisional (0-40.95)
    uint8_t  tdlas_peak_raw_     = 0; // x10, provisional (0-25.5)
    uint16_t tdlas_ratio_raw_    = 0; // x1000, provisional (0-1.023)
    uint8_t  round_robin_idx_    = 0; // 0-7, captured at construction from next_round_robin_idx_

    // Shared rotation state advanced via resetRotation()/advanceRotation().
    static uint8_t next_round_robin_idx_;

    // Slow / round-robin fields (period = 8)
    uint16_t opc_d500_           = 0;
    uint16_t opc_d700_           = 0;
    uint16_t opc_d1000_          = 0;
    uint16_t opc_d3000_          = 0;
    uint16_t opc_d5000_          = 0;
    uint16_t opc_d2500_          = 0; // spec "10000nm" slot
    uint16_t rs41_mag_xy_raw_    = 0; // counts + 1000
    uint16_t bemf_v_raw_         = 0; // x1000 V
    uint16_t tdlas_spec_1_raw_   = 0; // raw passthrough, provisional
    uint16_t tdlas_spec_2_raw_   = 0;
    uint16_t tdlas_spec_3_raw_   = 0;
    uint16_t tdlas_spec_4_raw_   = 0;
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
};

#endif /* RPUComm_H */
