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

// ---------------------------------------------------------------------------
// RPU report bit-field widths
// Shared between RPU (encoder) and RATCHuTS (decoder).
// Packet version 1 — 1066 bits = 134 bytes, big-endian.
// Carries every field measured once per tickMeasure() iteration, in the same
// order they are gathered. Sent in bulk over the docking connector (no LoRa
// size limit), so most fields use a fixed-point scale rather than raw floats;
// the TDLAS spectroscopy values (whose ranges aren't well characterised) are
// packed as raw IEEE-754 floats to avoid lossy guesses.
//
// Many of the following bit widths are shared for multiple fields; 
// for example, all the subsystem currents are 12 bits with the same scale factor
// ---------------------------------------------------------------------------
constexpr uint8_t  RPU_RPT_VERSION       = 1;
constexpr uint8_t  RPU_RPT_VER_BITS      = 4;    // packet format version
constexpr uint8_t  RPU_RPT_ID_BITS       = 16;   // board ID
constexpr uint8_t  RPU_RPT_TIME_BITS     = 32;   // elapsed_ms / ROPC_time / RS41 frame_count, raw
constexpr uint8_t  RPU_RPT_VOLT_BITS     = 12;   // V x100   (0–40.95 V)
constexpr uint8_t  RPU_RPT_CHGI_BITS     = 13;   // charge_i, mA  (0–8191 mA)
constexpr uint8_t  RPU_RPT_CURR_BITS     = 12;   // subsystem I, mA (0–4095 mA)
constexpr uint8_t  RPU_RPT_PWM_BITS      = 8;    // pump PWM (0–255)
constexpr uint8_t  RPU_RPT_LAT_BITS      = 21;   // (lat + 90)  x10000
constexpr uint8_t  RPU_RPT_LON_BITS      = 22;   // (lon + 180) x10000
constexpr uint8_t  RPU_RPT_ALT_BITS      = 16;   // altitude, m
constexpr uint8_t  RPU_RPT_SATS_BITS     = 5;    // satellite count
constexpr uint8_t  RPU_RPT_GPS_DATE_BITS = 19;   // DDMMYY
constexpr uint8_t  RPU_RPT_GPS_TIME_BITS = 25;   // HHMMSSCC
constexpr uint8_t  RPU_RPT_GPS_AGE_BITS  = 8;    // GPS fix age, s, clamped (0–255 s)
constexpr uint8_t  RPU_RPT_TEMP_BITS     = 12;   // (T + 100) x10  (-100.0 to 309.5 °C, 0.1 °C res)
constexpr uint8_t  RPU_RPT_HUM_BITS      = 10;   // RH x10   (0–102.3 %)
constexpr uint8_t  RPU_RPT_PRES_BITS     = 17;   // pressure x100 (0–1310.71 mb)
constexpr uint8_t  RPU_RPT_OPC_BITS      = 16;   // OPC bin counts, raw
constexpr uint8_t  RPU_RPT_ALARM_BITS    = 8;    // OPC alarm flags
constexpr uint8_t  RPU_RPT_TSEN_AIRT_BITS  = 12; // TSEN air temp, raw 12-bit A/D count (0–0xFFF)
constexpr uint8_t  RPU_RPT_TSEN_PTEMP_BITS = 24; // TSEN pressure-sensor temp, raw 24-bit count
constexpr uint8_t  RPU_RPT_TSEN_PRES_BITS  = 24; // TSEN pressure, raw 24-bit count
constexpr uint8_t  RPU_RPT_F32_BITS      = 32;   // raw IEEE-754 float (TDLAS)
constexpr uint8_t  RPU_RPT_INDX_BITS     = 8;    // TDLAS spectrum index
constexpr uint8_t  RPU_RPT_STATUS_BITS   = 8;    // RS41 module status/error
constexpr uint8_t  RPU_RPT_MAGXY_BITS    = 8;    // RS41 magnetometer X-Y, raw counts (-1000–1000), scaled to 0-255
constexpr size_t   RPU_RPT_BYTES         = 134;  // ceil(1066 / 8)

// ---------------------------------------------------------------------------
// RPUReport
// Holds one tickMeasure() sample (power, GPS, temperatures, OPC, TDLAS, RS41)
// and converts to/from the bit-packed wire format. Field order matches the
// order the values are gathered in tickMeasure().
// ---------------------------------------------------------------------------
class RPUReport {
public:
    RPUReport() = default;

    // Setters (engineering units -> packed encoding) ---------
    void setBoardId(uint16_t id);
    void setElapsedMs(uint32_t ms);
    void setBatV(float volts);
    void setVin(float volts);
    void setChargeI(float amps);
    void setV5V(float volts);
    void setPumpI(float milliamps);
    void setOpcI(float milliamps);
    void setTsenI(float milliamps);
    void setTdlasI(float milliamps);
    void setHeaterI(float milliamps);
    void setBemfV(float volts);
    void setPumpPwm(uint8_t pwm);
    void setLat(double degrees);
    void setLon(double degrees);
    void setAlt(float meters);
    void setSats(uint8_t count);
    void setGpsDate(uint32_t date);   // DDMMYY, as returned by TinyGPSDate::value()
    void setGpsTime(uint32_t time);   // HHMMSSCC, as returned by TinyGPSTime::value()
    void setGpsAge(uint32_t seconds);
    void setPcbT(float celsius);
    void setPumpT(float celsius);
    void setBatT(float celsius);
    void setOpcTime(uint32_t ms);
    void setOpcD300(uint16_t count);
    void setOpcD500(uint16_t count);
    void setOpcD700(uint16_t count);
    void setOpcD1000(uint16_t count);
    void setOpcD2000(uint16_t count);
    void setOpcD2500(uint16_t count);
    void setOpcD3000(uint16_t count);
    void setOpcD5000(uint16_t count);
    void setOpcAlarm(uint8_t alarm);
    void setTsenAirt(uint16_t raw);
    void setTsenPtemp(uint32_t raw);
    void setTsenPres(uint32_t raw);
    void setTdlasMrAvg(float value);
    void setTdlasBkg(float value);
    void setTdlasPeak(float value);
    void setTdlasRatio(float value);
    void setTdlasBatt(float volts);
    void setTdlasTherm1(float celsius);
    void setTdlasTherm2(float celsius);
    void setTdlasIndx(int8_t indx);
    void setTdlasSpec1(float value);
    void setTdlasSpec2(float value);
    void setTdlasSpec3(float value);
    void setTdlasSpec4(float value);
    void setRs41Valid(bool valid);
    void setRs41FrameCount(uint32_t count);
    void setRs41AirT(float celsius);
    void setRs41Humidity(float percent);
    void setRs41HSensorT(float celsius);
    void setRs41Pres(float millibar);
    void setRs41InternalT(float celsius);
    void setRs41ModuleStatus(uint8_t status);
    void setRs41ModuleError(uint8_t error);
    void setRs41PcbSupplyV(float volts);
    void setRs41Lsm303T(float celsius);
    void setRs41PcbHeaterOn(bool on);
    void setRs41MagXY(int32_t counts);

    // Getters (packed encoding -> engineering units) ---------
    uint16_t getBoardId()          const { return board_id_; }
    uint32_t getElapsedMs()        const { return elapsed_ms_; }
    float    getBatV()             const { return bat_v_raw_ / 100.0f; }
    float    getVin()              const { return vin_raw_ / 100.0f; }
    float    getChargeI()          const { return charge_i_raw_ / 1000.0f; }
    float    getV5V()              const { return v5v_raw_ / 100.0f; }
    float    getPumpI()            const { return (float)pump_i_raw_; }
    float    getOpcI()             const { return (float)opc_i_raw_; }
    float    getTsenI()            const { return (float)tsen_i_raw_; }
    float    getTdlasI()           const { return (float)tdlas_i_raw_; }
    float    getHeaterI()          const { return (float)heater_i_raw_; }
    float    getBemfV()            const { return bemf_v_raw_ / 100.0f; }
    uint8_t  getPumpPwm()          const { return pump_pwm_; }
    double   getLat()              const { return (lat_raw_ / 10000.0) - 90.0; }
    double   getLon()              const { return (lon_raw_ / 10000.0) - 180.0; }
    float    getAlt()              const { return (float)alt_raw_; }
    uint8_t  getSats()              const { return sats_; }
    uint32_t getGpsDate()          const { return gps_date_; }
    uint32_t getGpsTime()          const { return gps_time_; }
    uint32_t getGpsAge()           const { return gps_age_s_; }
    float    getPcbT()             const { return (pcb_t_raw_ / 10.0f) - 100.0f; }
    float    getPumpT()            const { return (pump_t_raw_ / 10.0f) - 100.0f; }
    float    getBatT()             const { return (bat_t_raw_ / 10.0f) - 100.0f; }
    uint32_t getOpcTime()          const { return opc_time_; }
    uint16_t getOpcD300()          const { return opc_d300_; }
    uint16_t getOpcD500()          const { return opc_d500_; }
    uint16_t getOpcD700()          const { return opc_d700_; }
    uint16_t getOpcD1000()         const { return opc_d1000_; }
    uint16_t getOpcD2000()         const { return opc_d2000_; }
    uint16_t getOpcD2500()         const { return opc_d2500_; }
    uint16_t getOpcD3000()         const { return opc_d3000_; }
    uint16_t getOpcD5000()         const { return opc_d5000_; }
    uint8_t  getOpcAlarm()         const { return opc_alarm_; }
    uint16_t getTsenAirt()         const { return tsen_airt_raw_; }
    uint32_t getTsenPtemp()        const { return tsen_ptemp_raw_; }
    uint32_t getTsenPres()         const { return tsen_pres_raw_; }
    float    getTdlasMrAvg()       const { return tdlas_mr_avg_; }
    float    getTdlasBkg()         const { return tdlas_bkg_; }
    float    getTdlasPeak()        const { return tdlas_peak_; }
    float    getTdlasRatio()       const { return tdlas_ratio_; }
    float    getTdlasBatt()        const { return tdlas_batt_; }
    float    getTdlasTherm1()      const { return tdlas_therm_1_; }
    float    getTdlasTherm2()      const { return tdlas_therm_2_; }
    int8_t   getTdlasIndx()        const { return tdlas_indx_; }
    float    getTdlasSpec1()       const { return tdlas_spec_1_; }
    float    getTdlasSpec2()       const { return tdlas_spec_2_; }
    float    getTdlasSpec3()       const { return tdlas_spec_3_; }
    float    getTdlasSpec4()       const { return tdlas_spec_4_; }
    bool     getRs41Valid()        const { return rs41_valid_; }
    uint32_t getRs41FrameCount()   const { return rs41_frame_count_; }
    float    getRs41AirT()         const { return (rs41_air_t_raw_ / 10.0f) - 100.0f; }
    float    getRs41Humidity()     const { return rs41_humidity_raw_ / 10.0f; }
    float    getRs41HSensorT()     const { return (rs41_hsensor_t_raw_ / 10.0f) - 100.0f; }
    float    getRs41Pres()         const { return rs41_pres_raw_ / 100.0f; }
    float    getRs41InternalT()    const { return (rs41_internal_t_raw_ / 10.0f) - 100.0f; }
    uint8_t  getRs41ModuleStatus() const { return rs41_module_status_; }
    uint8_t  getRs41ModuleError()  const { return rs41_module_error_; }
    float    getRs41PcbSupplyV()   const { return rs41_pcb_supply_v_raw_ / 100.0f; }
    float    getRs41Lsm303T()      const { return (rs41_lsm303_t_raw_ / 10.0f) - 100.0f; }
    bool     getRs41PcbHeaterOn()  const { return rs41_pcb_heater_on_; }
    int32_t  getRs41MagXY()        const { return (int32_t)((rs41_mag_xy_ / 255.0f) * 2000.0f - 1000.0f); }

    // Byte-level pack / unpack using the bit-field widths above
    bool encode(uint8_t* buf, size_t buf_size) const;
    bool decode(const uint8_t* buf, size_t buf_size);

    // JSON serialisation in engineering units.
    String toJSON() const;

private:
    uint16_t board_id_              = 0;
    uint32_t elapsed_ms_            = 0;
    uint16_t bat_v_raw_             = 0; // x100 V
    uint16_t vin_raw_               = 0; // x100 V
    uint16_t charge_i_raw_          = 0; // mA
    uint16_t v5v_raw_               = 0; // x100 V
    uint16_t pump_i_raw_            = 0; // mA
    uint16_t opc_i_raw_             = 0; // mA
    uint16_t tsen_i_raw_            = 0; // mA
    uint16_t tdlas_i_raw_           = 0; // mA
    uint16_t heater_i_raw_          = 0; // mA
    uint16_t bemf_v_raw_            = 0; // x100 V
    uint8_t  pump_pwm_              = 0;
    uint32_t lat_raw_               = 0; // (lat + 90)  x10000
    uint32_t lon_raw_               = 0; // (lon + 180) x10000
    uint16_t alt_raw_               = 0; // m
    uint8_t  sats_                  = 0;
    uint32_t gps_date_              = 0; // DDMMYY
    uint32_t gps_time_              = 0; // HHMMSSCC
    uint32_t gps_age_s_             = 0;
    uint16_t pcb_t_raw_             = 0; // (T + 100) x10
    uint16_t pump_t_raw_            = 0; // (T + 100) x10
    uint16_t bat_t_raw_             = 0; // (T + 100) x10
    uint32_t opc_time_              = 0;
    uint16_t opc_d300_               = 0;
    uint16_t opc_d500_               = 0;
    uint16_t opc_d700_               = 0;
    uint16_t opc_d1000_              = 0;
    uint16_t opc_d2000_              = 0;
    uint16_t opc_d2500_              = 0;
    uint16_t opc_d3000_              = 0;
    uint16_t opc_d5000_              = 0;
    uint8_t  opc_alarm_              = 0;
    uint16_t tsen_airt_raw_          = 0; // raw 12-bit A/D count
    uint32_t tsen_ptemp_raw_         = 0; // raw 24-bit count
    uint32_t tsen_pres_raw_          = 0; // raw 24-bit count
    float    tdlas_mr_avg_           = 0.0f;
    float    tdlas_bkg_              = 0.0f;
    float    tdlas_peak_             = 0.0f;
    float    tdlas_ratio_            = 0.0f;
    float    tdlas_batt_             = 0.0f;
    float    tdlas_therm_1_          = 0.0f;
    float    tdlas_therm_2_          = 0.0f;
    int8_t   tdlas_indx_             = 0;
    float    tdlas_spec_1_           = 0.0f;
    float    tdlas_spec_2_           = 0.0f;
    float    tdlas_spec_3_           = 0.0f;
    float    tdlas_spec_4_           = 0.0f;
    bool     rs41_valid_             = false;
    uint32_t rs41_frame_count_       = 0;
    uint16_t rs41_air_t_raw_         = 0; // (T + 100) x10
    uint16_t rs41_humidity_raw_      = 0; // x10 %
    uint16_t rs41_hsensor_t_raw_     = 0; // (T + 100) x10
    uint32_t rs41_pres_raw_          = 0; // x100 mb
    uint16_t rs41_internal_t_raw_    = 0; // (T + 100) x10
    uint8_t  rs41_module_status_     = 0;
    uint8_t  rs41_module_error_      = 0;
    uint16_t rs41_pcb_supply_v_raw_  = 0; // x100 V
    uint16_t rs41_lsm303_t_raw_      = 0; // (T + 100) x10
    bool     rs41_pcb_heater_on_     = false;
    uint8_t  rs41_mag_xy_            = 0; // raw counts (-1000–1000), scaled to 0-255
};

#endif /* RPUComm_H */
