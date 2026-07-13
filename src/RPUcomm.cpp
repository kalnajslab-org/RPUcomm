/*
 *  RPUComm.cpp
 *  Derived from PUcomm.cpp
 *  
 *  This file implements an Arduino library (C++ class) that implements the communication
 *  between the RATCHuTS and RPU. The class inherits its protocol from the SerialComm
 *  class.
 */

#include "RPUcomm.h"
#include <etl/bit_stream.h>

RPUComm::RPUComm(Stream * serial_port)
    : SerialComm(serial_port)
{
}

// RATCHuTS -> RPU (with params) ---------------------------

// RATCHuTS -> RPU: 2026 commands -------------------------

bool RPUComm::TX_GoMeasure(int32_t duration, int32_t rate, float bat_temp, int8_t opc_power, int8_t tdlas_power, int8_t tsen_power, int8_t rs41_power)
{
    if (!Add_int32(duration))    return false;
    if (!Add_int32(rate))        return false;
    if (!Add_float(bat_temp))    return false;
    if (!Add_int8(opc_power))    return false;
    if (!Add_int8(tdlas_power))  return false;
    if (!Add_int8(tsen_power))   return false;
    if (!Add_int8(rs41_power))   return false;
    TX_ASCII(RPU_GO_MEASURE);
    return true;
}

bool RPUComm::RX_GoMeasure(int32_t * duration, int32_t * rate, float * bat_temp, int8_t * opc_power, int8_t * tdlas_power, int8_t * tsen_power, int8_t * rs41_power)
{
    int32_t t, r;
    float   b;
    int8_t  o, d, s, g;
    if (!Get_int32(&t)) return false;
    if (!Get_int32(&r)) return false;
    if (!Get_float(&b)) return false;
    if (!Get_int8(&o))  return false;
    if (!Get_int8(&d))  return false;
    if (!Get_int8(&s))  return false;
    if (!Get_int8(&g))  return false;
    *duration    = t;
    *rate        = r;
    *bat_temp    = b;
    *opc_power   = o;
    *tdlas_power = d;
    *tsen_power  = s;
    *rs41_power  = g;
    return true;
}

bool RPUComm::TX_GoStandby(float bat_temp)
{
    if (!Add_float(bat_temp)) return false;
    TX_ASCII(RPU_GO_STANDBY);
    return true;
}

bool RPUComm::RX_GoStandby(float * bat_temp)
{
    float b;
    if (!Get_float(&b)) return false;
    *bat_temp = b;
    return true;
}

bool RPUComm::TX_SetStatusRate(uint32_t interval)
{
    if (!Add_uint32(interval)) return false;
    TX_ASCII(RPU_SET_STATUS_RATE);
    return true;
}

bool RPUComm::RX_SetStatusRate(uint32_t * interval)
{
    uint32_t temp;
    if (!Get_uint32(&temp)) return false;
    *interval = temp;
    return true;
}

bool RPUComm::TX_Status(const char * json)
{
    uint16_t length = (uint16_t) strlen(json);

    AssignBinaryTXBuffer((uint8_t *) json, length, length);

    return TX_Bin(RPU_STATUS);
}

bool RPUComm::RX_Status(char * json, uint16_t buffer_size)
{
    if (binary_rx.bin_length >= buffer_size) return false;

    memcpy(json, binary_rx.bin_buffer, binary_rx.bin_length);
    json[binary_rx.bin_length] = '\0';

    return true;
}

// -- RPU to RATCHuTS error string

bool RPUComm::TX_Error(const char * error)
{
    //if (Add_string(error)) return false;

    TX_String(RPU_ERROR,error);

    return true;
}

bool RPUComm::RX_Error(char * error, uint8_t buffer_size)
{
    return Get_string(error, buffer_size);
}

// ---------------------------------------------------------------------------
// RPUPacket
// ---------------------------------------------------------------------------

void RPUPacket::setBoardId(uint16_t id)        { board_id_    = id; }
void RPUPacket::setState(uint8_t state)        { state_       = state; }
void RPUPacket::setWdtCount(uint8_t count)     { wdt_count_   = count; }
void RPUPacket::setBufferedRecords(uint16_t count) { buffered_records_ = count; }
void RPUPacket::setVin(float volts)            { vin_raw_     = (uint8_t)constrain((int)(volts * 10.0f), 0, 255); }
void RPUPacket::setV5V(float volts)            { v5v_raw_     = (uint8_t)constrain((int)(volts * 10.0f), 0, 255); }
void RPUPacket::setBatV(float volts)           { bat_v_raw_   = (uint8_t)constrain((int)(volts * 10.0f), 0, 255); }
void RPUPacket::setHeaterDuty(uint8_t percent) { heater_duty_ = (uint8_t)constrain((int)percent, 0, 100); }
void RPUPacket::setChgI(float amps)            { chg_i_raw_   = (uint8_t)constrain((int)(amps * 20.0f), 0, 127); }
void RPUPacket::setBatT(float celsius)         { bat_t_raw_   = (uint16_t)constrain((int)((celsius + 100.0f) * 2.0f), 0, 511); }
void RPUPacket::setPcbT(float celsius)         { pcb_t_raw_   = (uint16_t)constrain((int)((celsius + 100.0f) * 2.0f), 0, 511); }
void RPUPacket::setPumpI(float milliamps)      { pump_i_raw_   = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setOpcI(float milliamps)       { opc_i_raw_    = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setTsenI(float milliamps)      { tsen_i_raw_   = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setTdlasI(float milliamps)     { tdlas_i_raw_  = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setHeaterI(float milliamps)    { heater_i_raw_ = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setLat(double degrees)         { lat_raw_      = (uint32_t)constrain((int)((degrees + 90.0)  * 10000.0), 0, 1800000); }
void RPUPacket::setLon(double degrees)         { lon_raw_      = (uint32_t)constrain((int)((degrees + 180.0) * 10000.0), 0, 3600000); }
void RPUPacket::setAlt(float meters)           { alt_raw_      = (uint16_t)constrain((int)meters, 0, 65535); }
void RPUPacket::setSats(uint8_t count)         { sats_         = (uint8_t)constrain((int)count, 0, 31); }
void RPUPacket::setGpsDate(uint32_t date)      { gps_date_     = (uint32_t)constrain((int)date, 0, 524287); }
void RPUPacket::setGpsTime(uint32_t time)      { gps_time_     = (uint32_t)constrain((int)time, 0, 33554431); }

void RPUPacket::setVer(const char * ver)
{
    memset(ver_, 0, sizeof(ver_));
    if (ver != nullptr) {
        strncpy(ver_, ver, sizeof(ver_) - 1);
    }
}

bool RPUPacket::encode(uint8_t * buf, size_t buf_size) const
{
    if (buf_size < RPU_PKT_BYTES) return false;

    memset(buf, 0, RPU_PKT_BYTES);
    etl::bit_stream_writer bsw(buf, RPU_PKT_BYTES, etl::bit_order::msb_first);

    bsw.write_unchecked<uint8_t> (RPU_PKT_VERSION,  RPU_PKT_VER_BITS);
    bsw.write_unchecked<uint16_t>(board_id_,        RPU_PKT_ID_BITS);
    bsw.write_unchecked<uint8_t> (state_,           RPU_PKT_STATE_BITS);
    bsw.write_unchecked<uint8_t> (wdt_count_,       RPU_PKT_WDT_BITS);
    bsw.write_unchecked<uint16_t>(buffered_records_, RPU_PKT_BUFREC_BITS);
    bsw.write_unchecked<uint8_t> (vin_raw_,         RPU_PKT_VIN_BITS);
    bsw.write_unchecked<uint8_t> (v5v_raw_,         RPU_PKT_V5_BITS);
    bsw.write_unchecked<uint8_t> (bat_v_raw_,       RPU_PKT_BATV_BITS);
    bsw.write_unchecked<uint8_t> (heater_duty_,     RPU_PKT_DUTY_BITS);
    bsw.write_unchecked<uint8_t> (chg_i_raw_,       RPU_PKT_CHGI_BITS);
    bsw.write_unchecked<uint16_t>(bat_t_raw_,       RPU_PKT_TEMP_BITS);
    bsw.write_unchecked<uint16_t>(pcb_t_raw_,       RPU_PKT_TEMP_BITS);
    bsw.write_unchecked<uint16_t>(pump_i_raw_,      RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(opc_i_raw_,       RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(tsen_i_raw_,      RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(tdlas_i_raw_,     RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(heater_i_raw_,    RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint32_t>(lat_raw_,         RPU_PKT_LAT_BITS);
    bsw.write_unchecked<uint32_t>(lon_raw_,         RPU_PKT_LON_BITS);
    bsw.write_unchecked<uint16_t>(alt_raw_,         RPU_PKT_ALT_BITS);
    bsw.write_unchecked<uint8_t> (sats_,            RPU_PKT_SATS_BITS);
    bsw.write_unchecked<uint32_t>(gps_date_,        RPU_PKT_GPS_DATE_BITS);
    bsw.write_unchecked<uint32_t>(gps_time_,        RPU_PKT_GPS_TIME_BITS);

    for (size_t i = 0; i < RPU_PKT_FW_VER_LEN; ++i) {
        bsw.write_unchecked<uint8_t>((uint8_t)ver_[i], 8);
    }

    return true;
}

bool RPUPacket::decode(const uint8_t * buf, size_t buf_size)
{
    if (buf_size < RPU_PKT_BYTES) return false;

    etl::bit_stream_reader bsr(buf, RPU_PKT_BYTES, etl::bit_order::msb_first);

    bsr.read_unchecked<uint8_t>(RPU_PKT_VER_BITS);  // packet version, not stored

    board_id_     = bsr.read_unchecked<uint16_t>(RPU_PKT_ID_BITS);
    state_        = bsr.read_unchecked<uint8_t> (RPU_PKT_STATE_BITS);
    wdt_count_    = bsr.read_unchecked<uint8_t> (RPU_PKT_WDT_BITS);
    buffered_records_ = bsr.read_unchecked<uint16_t>(RPU_PKT_BUFREC_BITS);
    vin_raw_      = bsr.read_unchecked<uint8_t> (RPU_PKT_VIN_BITS);
    v5v_raw_      = bsr.read_unchecked<uint8_t> (RPU_PKT_V5_BITS);
    bat_v_raw_    = bsr.read_unchecked<uint8_t> (RPU_PKT_BATV_BITS);
    heater_duty_  = bsr.read_unchecked<uint8_t> (RPU_PKT_DUTY_BITS);
    chg_i_raw_    = bsr.read_unchecked<uint8_t> (RPU_PKT_CHGI_BITS);
    bat_t_raw_    = bsr.read_unchecked<uint16_t>(RPU_PKT_TEMP_BITS);
    pcb_t_raw_    = bsr.read_unchecked<uint16_t>(RPU_PKT_TEMP_BITS);
    pump_i_raw_   = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    opc_i_raw_    = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    tsen_i_raw_   = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    tdlas_i_raw_  = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    heater_i_raw_ = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    lat_raw_      = bsr.read_unchecked<uint32_t>(RPU_PKT_LAT_BITS);
    lon_raw_      = bsr.read_unchecked<uint32_t>(RPU_PKT_LON_BITS);
    alt_raw_      = bsr.read_unchecked<uint16_t>(RPU_PKT_ALT_BITS);
    sats_         = bsr.read_unchecked<uint8_t> (RPU_PKT_SATS_BITS);
    gps_date_     = bsr.read_unchecked<uint32_t>(RPU_PKT_GPS_DATE_BITS);
    gps_time_     = bsr.read_unchecked<uint32_t>(RPU_PKT_GPS_TIME_BITS);

    for (size_t i = 0; i < RPU_PKT_FW_VER_LEN; ++i) {
        ver_[i] = (char)bsr.read_unchecked<uint8_t>(8);
    }
    ver_[RPU_PKT_FW_VER_LEN - 1] = '\0';

    return true;
}

String RPUPacket::toJSON() const
{
    static const char * state_names[] = { "STANDBY", "MEASURE", "ERROR" };
    const char * state_str = (state_ < (sizeof(state_names) / sizeof(state_names[0])))
                             ? state_names[state_]
                             : "UNKNOWN";

    char buf[350];
    snprintf(buf, sizeof(buf),
        "{\"id\":\"%04X\",\"ver\":\"%s\",\"state\":\"%s\",\"wdt_n\":%u,\"buf_rec\":%u,"
        "\"vin\":%.1f,\"v5\":%.1f,\"bat_v\":%.1f,\"bat_duty\":%u,\"chg_i\":%.1f,"
        "\"bat_t\":%.1f,\"pcb_t\":%.1f,"
        "\"pump_i\":%.0f,\"opc_i\":%.0f,\"tsen_i\":%.0f,\"tdlas_i\":%.0f,\"heater_i\":%.0f,"
        "\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"sats\":%u,"
        "\"date\":%lu,\"time\":%lu}",
        board_id_, ver_, state_str, wdt_count_, buffered_records_,
        getVin(), getV5V(), getBatV(), heater_duty_, getChgI(),
        getBatT(), getPcbT(),
        getPumpI(), getOpcI(), getTsenI(), getTdlasI(), getHeaterI(),
        getLat(), getLon(), getAlt(), sats_,
        (unsigned long)gps_date_, (unsigned long)gps_time_);

    return String(buf);
}

// ---------------------------------------------------------------------------
// RPURecord
// ---------------------------------------------------------------------------

RPURecord::RPURecord() : round_robin_idx_(0)
{
}

void RPURecord::resetRotation()
{
    round_robin_idx_ = 0;
}

void RPURecord::advanceRotation()
{
    round_robin_idx_ = (round_robin_idx_ + 1) % 8;
}

// Fast fields (period = 1) ----------------------------------
void RPURecord::setElapsedS(uint32_t seconds)  { elapsed_s_     = (uint16_t)constrain((long)seconds, 0L, 65535L); }
void RPURecord::setAlt(float meters)           { alt_raw_       = (uint16_t)constrain((int)meters, 0, 65535); }
void RPURecord::setLatDelta(double degrees)    { lat_delta_raw_ = (int16_t)constrain((long)(degrees * 50000.0), -32768L, 32767L); }
void RPURecord::setLonDelta(double degrees)    { lon_delta_raw_ = (int16_t)constrain((long)(degrees * 50000.0), -32768L, 32767L); }
void RPURecord::setSats(uint8_t count)         { sats_          = (uint8_t)constrain((int)count, 0, 15); }
void RPURecord::setGpsAge(uint32_t seconds)    { gps_age_s_     = (uint8_t)constrain((int)seconds, 0, 15); }
void RPURecord::setOpcD300(uint16_t count)     { opc_d300_      = count; }
void RPURecord::setOpcD2000(uint16_t count)    { opc_d2000_     = count; }
void RPURecord::setTsenAirt(uint16_t raw)      { tsen_airt_raw_  = (uint16_t)constrain((int)raw, 0, 0xFFF); }
void RPURecord::setTsenPres(uint32_t raw)      { tsen_pres_raw_  = (uint16_t)(constrain((long)raw, 0L, 0xFFFFFFL) >> 8); }
void RPURecord::setTsenPtemp(uint32_t raw)     { tsen_ptemp_raw_ = (uint16_t)(constrain((long)raw, 0L, 0xFFFFFFL) >> 8); }
void RPURecord::setRs41AirT(float celsius)     { rs41_air_t_raw_     = (uint16_t)constrain((int)((celsius + 100.0f) * 100.0f), 0, 65535); }
void RPURecord::setRs41Pres(float millibar)    { rs41_pres_raw_      = (uint16_t)constrain((int)(millibar * 10.0f), 0, 65535); }
void RPURecord::setRs41Humidity(float percent) { rs41_humidity_raw_  = (uint16_t)constrain((int)(percent * 100.0f), 0, 65535); }
void RPURecord::setRs41HSensorT(float celsius) { rs41_hsensor_t_raw_ = (uint16_t)constrain((int)((celsius + 100.0f) * 100.0f), 0, 65535); }
void RPURecord::setTdlasMrAvg(float value)     { tdlas_mr_avg_raw_ = (uint16_t)constrain((int)(value * 10.0f), 0, 1023); }
void RPURecord::setTdlasBkg(float value)       { tdlas_bkg_raw_    = (uint16_t)constrain((int)(value * 100.0f), 0, 4095); }
void RPURecord::setTdlasPeak(float value)      { tdlas_peak_raw_   = (uint8_t)constrain((int)(value * 10.0f), 0, 255); }
void RPURecord::setTdlasRatio(float value)     { tdlas_ratio_raw_  = (uint16_t)constrain((int)(value * 1000.0f), 0, 1023); }

// Slow / round-robin fields (period = 8) ---------------------
void RPURecord::setOpcD500(uint16_t count)     { opc_d500_  = count; }
void RPURecord::setOpcD700(uint16_t count)     { opc_d700_  = count; }
void RPURecord::setOpcD1000(uint16_t count)    { opc_d1000_ = count; }
void RPURecord::setOpcD3000(uint16_t count)    { opc_d3000_ = count; }
void RPURecord::setOpcD5000(uint16_t count)    { opc_d5000_ = count; }
void RPURecord::setOpcD2500(uint16_t count)    { opc_d2500_ = count; }
void RPURecord::setRs41Hdg(float degrees)      { rs41_hdg_raw_ = (uint16_t)constrain((int)(degrees * 100.0f), 0, 36000); }
void RPURecord::setBemfV(float volts)          { bemf_v_raw_      = (uint16_t)constrain((int)(volts * 1000.0f), 0, 65535); }
void RPURecord::setTdlasSpec1(float value)     { tdlas_spec_1_raw_ = (uint16_t)constrain((int)value, 0, 65535); }
void RPURecord::setTdlasSpec2(float value)     { tdlas_spec_2_raw_ = (uint16_t)constrain((int)value, 0, 65535); }
void RPURecord::setTdlasSpec3(float value)     { tdlas_spec_3_raw_ = (uint16_t)constrain((int)value, 0, 65535); }
void RPURecord::setTdlasSpec4(float value)     { tdlas_spec_4_raw_ = (uint16_t)constrain((int)value, 0, 65535); }
void RPURecord::setTsenI(float milliamps)      { tsen_i_raw_  = (uint8_t)constrain((int)(milliamps / 4.0f), 0, 255); }
void RPURecord::setOpcI(float milliamps)       { opc_i_raw_   = (uint8_t)constrain((int)(milliamps / 4.0f), 0, 255); }
void RPURecord::setPumpI(float milliamps)      { pump_i_raw_  = (uint8_t)constrain((int)(milliamps / 4.0f), 0, 255); }
void RPURecord::setTdlasI(float milliamps)     { tdlas_i_raw_ = (uint8_t)constrain((int)(milliamps / 4.0f), 0, 255); }
void RPURecord::setV5V(float volts)            { v5v_raw_     = (uint8_t)constrain((int)(volts * 50.0f), 0, 255); }
void RPURecord::setBatT(float celsius)         { bat_t_raw_   = (uint8_t)constrain((int)(celsius + 100.0f), 0, 255); }
void RPURecord::setPumpT(float celsius)        { pump_t_raw_  = (uint8_t)constrain((int)(celsius + 100.0f), 0, 255); }
void RPURecord::setPcbT(float celsius)         { pcb_t_raw_   = (uint8_t)constrain((int)(celsius + 100.0f), 0, 255); }
void RPURecord::setBatV(float volts)           { bat_v_raw_   = (uint16_t)constrain((int)(volts * 100.0f), 0, 4095); }
void RPURecord::setHeaterStat(uint8_t status)  { heater_stat_ = (uint8_t)constrain((int)status, 0, 15); }

bool RPURecord::encode(uint8_t * buf, size_t buf_size) const
{
    if (buf_size < RPU_RECORD_BYTES) return false;

    memset(buf, 0, RPU_RECORD_BYTES);
    etl::bit_stream_writer bsw(buf, RPU_RECORD_BYTES, etl::bit_order::msb_first);

    bsw.write_unchecked<uint8_t> (RPU_RPT_VERSION, RPU_RPT_VER_BITS);

    // Fast fields (period = 1)
    bsw.write_unchecked<uint8_t> (round_robin_idx_,  RPU_RPT_RR_IDX_BITS);
    bsw.write_unchecked<uint16_t>(elapsed_s_,        RPU_RPT_ELAPSED_BITS);
    bsw.write_unchecked<uint16_t>(alt_raw_,          RPU_RPT_ALT_BITS);
    bsw.write_unchecked<int16_t> (lat_delta_raw_,    RPU_RPT_GPS_DELTA_BITS);
    bsw.write_unchecked<int16_t> (lon_delta_raw_,    RPU_RPT_GPS_DELTA_BITS);
    bsw.write_unchecked<uint8_t> (sats_,             RPU_RPT_SATS_BITS);
    bsw.write_unchecked<uint8_t> (gps_age_s_,        RPU_RPT_GPS_AGE_BITS);
    bsw.write_unchecked<uint16_t>(opc_d300_,         RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(opc_d2000_,        RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(tsen_airt_raw_,    RPU_RPT_TSEN_BITS);
    bsw.write_unchecked<uint16_t>(tsen_pres_raw_,    RPU_RPT_TSEN_BITS);
    bsw.write_unchecked<uint16_t>(tsen_ptemp_raw_,   RPU_RPT_TSEN_BITS);
    bsw.write_unchecked<uint16_t>(rs41_air_t_raw_,   RPU_RPT_RS41_T_BITS);
    bsw.write_unchecked<uint16_t>(rs41_pres_raw_,    RPU_RPT_RS41_P_BITS);
    bsw.write_unchecked<uint16_t>(rs41_humidity_raw_,RPU_RPT_RS41_RH_BITS);
    bsw.write_unchecked<uint16_t>(rs41_hsensor_t_raw_, RPU_RPT_RS41_T_BITS);
    bsw.write_unchecked<uint16_t>(tdlas_mr_avg_raw_, RPU_RPT_TDLAS_VMR_BITS);
    bsw.write_unchecked<uint16_t>(tdlas_bkg_raw_,    RPU_RPT_TDLAS_BKG_BITS);
    bsw.write_unchecked<uint8_t> (tdlas_peak_raw_,   RPU_RPT_TDLAS_PEAK_BITS);
    bsw.write_unchecked<uint16_t>(tdlas_ratio_raw_,  RPU_RPT_TDLAS_RATIO_BITS);

    // Slow / round-robin fields (period = 8) — one 40-bit slot per record
    switch (round_robin_idx_) {
        case 0:
            bsw.write_unchecked<uint16_t>(opc_d500_, RPU_RPT_OPC_BITS);
            bsw.write_unchecked<uint16_t>(opc_d700_, RPU_RPT_OPC_BITS);
            bsw.write_unchecked<uint8_t> (0,         RPU_RPT_SLOT_PAD_BITS);
            break;
        case 1:
            bsw.write_unchecked<uint16_t>(opc_d1000_, RPU_RPT_OPC_BITS);
            bsw.write_unchecked<uint16_t>(opc_d2500_, RPU_RPT_OPC_BITS);
            bsw.write_unchecked<uint8_t> (0,          RPU_RPT_SLOT_PAD_BITS);
            break;
        case 2:
            bsw.write_unchecked<uint16_t>(opc_d3000_, RPU_RPT_OPC_BITS);
            bsw.write_unchecked<uint16_t>(opc_d5000_, RPU_RPT_OPC_BITS);
            bsw.write_unchecked<uint8_t> (0,          RPU_RPT_SLOT_PAD_BITS);
            break;
        case 3:
            bsw.write_unchecked<uint16_t>(rs41_hdg_raw_, RPU_RPT_HDG_BITS);
            bsw.write_unchecked<uint16_t>(bemf_v_raw_,      RPU_RPT_BEMF_BITS);
            bsw.write_unchecked<uint8_t> (0,                RPU_RPT_SLOT_PAD_BITS);
            break;
        case 4:
            bsw.write_unchecked<uint16_t>(tdlas_spec_1_raw_, RPU_RPT_SPEC_BITS);
            bsw.write_unchecked<uint16_t>(tdlas_spec_2_raw_, RPU_RPT_SPEC_BITS);
            bsw.write_unchecked<uint8_t> (0,                 RPU_RPT_SLOT_PAD_BITS);
            break;
        case 5:
            bsw.write_unchecked<uint16_t>(tdlas_spec_3_raw_, RPU_RPT_SPEC_BITS);
            bsw.write_unchecked<uint16_t>(tdlas_spec_4_raw_, RPU_RPT_SPEC_BITS);
            bsw.write_unchecked<uint8_t> (0,                 RPU_RPT_SLOT_PAD_BITS);
            break;
        case 6:
            bsw.write_unchecked<uint8_t>(tsen_i_raw_,  RPU_RPT_HKCURR_BITS);
            bsw.write_unchecked<uint8_t>(opc_i_raw_,   RPU_RPT_HKCURR_BITS);
            bsw.write_unchecked<uint8_t>(pump_i_raw_,  RPU_RPT_HKCURR_BITS);
            bsw.write_unchecked<uint8_t>(tdlas_i_raw_, RPU_RPT_HKCURR_BITS);
            bsw.write_unchecked<uint8_t>(v5v_raw_,     RPU_RPT_V5V_BITS);
            break;
        case 7:
            bsw.write_unchecked<uint8_t> (bat_t_raw_,  RPU_RPT_HKTEMP_BITS);
            bsw.write_unchecked<uint8_t> (pump_t_raw_, RPU_RPT_HKTEMP_BITS);
            bsw.write_unchecked<uint8_t> (pcb_t_raw_,  RPU_RPT_HKTEMP_BITS);
            bsw.write_unchecked<uint16_t>(bat_v_raw_,  RPU_RPT_VOLT_BITS);
            bsw.write_unchecked<uint8_t> (heater_stat_,RPU_RPT_HEATER_BITS);
            break;
    }

    return true;
}

bool RPURecord::decode(const uint8_t * buf, size_t buf_size)
{
    if (buf_size < RPU_RECORD_BYTES) return false;

    etl::bit_stream_reader bsr(buf, RPU_RECORD_BYTES, etl::bit_order::msb_first);

    bsr.read_unchecked<uint8_t>(RPU_RPT_VER_BITS);  // packet version, not stored

    // Fast fields (period = 1)
    round_robin_idx_    = bsr.read_unchecked<uint8_t> (RPU_RPT_RR_IDX_BITS);
    elapsed_s_          = bsr.read_unchecked<uint16_t>(RPU_RPT_ELAPSED_BITS);
    alt_raw_            = bsr.read_unchecked<uint16_t>(RPU_RPT_ALT_BITS);
    lat_delta_raw_      = bsr.read_unchecked<int16_t> (RPU_RPT_GPS_DELTA_BITS);
    lon_delta_raw_      = bsr.read_unchecked<int16_t> (RPU_RPT_GPS_DELTA_BITS);
    sats_               = bsr.read_unchecked<uint8_t> (RPU_RPT_SATS_BITS);
    gps_age_s_          = bsr.read_unchecked<uint8_t> (RPU_RPT_GPS_AGE_BITS);
    opc_d300_           = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_d2000_          = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    tsen_airt_raw_      = bsr.read_unchecked<uint16_t>(RPU_RPT_TSEN_BITS);
    tsen_pres_raw_      = bsr.read_unchecked<uint16_t>(RPU_RPT_TSEN_BITS);
    tsen_ptemp_raw_     = bsr.read_unchecked<uint16_t>(RPU_RPT_TSEN_BITS);
    rs41_air_t_raw_     = bsr.read_unchecked<uint16_t>(RPU_RPT_RS41_T_BITS);
    rs41_pres_raw_      = bsr.read_unchecked<uint16_t>(RPU_RPT_RS41_P_BITS);
    rs41_humidity_raw_  = bsr.read_unchecked<uint16_t>(RPU_RPT_RS41_RH_BITS);
    rs41_hsensor_t_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_RS41_T_BITS);
    tdlas_mr_avg_raw_   = bsr.read_unchecked<uint16_t>(RPU_RPT_TDLAS_VMR_BITS);
    tdlas_bkg_raw_      = bsr.read_unchecked<uint16_t>(RPU_RPT_TDLAS_BKG_BITS);
    tdlas_peak_raw_     = bsr.read_unchecked<uint8_t> (RPU_RPT_TDLAS_PEAK_BITS);
    tdlas_ratio_raw_    = bsr.read_unchecked<uint16_t>(RPU_RPT_TDLAS_RATIO_BITS);

    // Slow / round-robin fields (period = 8) — one 40-bit slot per record
    switch (round_robin_idx_) {
        case 0:
            opc_d500_ = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
            opc_d700_ = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
            bsr.read_unchecked<uint8_t>(RPU_RPT_SLOT_PAD_BITS);
            break;
        case 1:
            opc_d1000_ = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
            opc_d2500_ = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
            bsr.read_unchecked<uint8_t>(RPU_RPT_SLOT_PAD_BITS);
            break;
        case 2:
            opc_d3000_ = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
            opc_d5000_ = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
            bsr.read_unchecked<uint8_t>(RPU_RPT_SLOT_PAD_BITS);
            break;
        case 3:
            rs41_hdg_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_HDG_BITS);
            bemf_v_raw_      = bsr.read_unchecked<uint16_t>(RPU_RPT_BEMF_BITS);
            bsr.read_unchecked<uint8_t>(RPU_RPT_SLOT_PAD_BITS);
            break;
        case 4:
            tdlas_spec_1_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_SPEC_BITS);
            tdlas_spec_2_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_SPEC_BITS);
            bsr.read_unchecked<uint8_t>(RPU_RPT_SLOT_PAD_BITS);
            break;
        case 5:
            tdlas_spec_3_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_SPEC_BITS);
            tdlas_spec_4_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_SPEC_BITS);
            bsr.read_unchecked<uint8_t>(RPU_RPT_SLOT_PAD_BITS);
            break;
        case 6:
            tsen_i_raw_  = bsr.read_unchecked<uint8_t>(RPU_RPT_HKCURR_BITS);
            opc_i_raw_   = bsr.read_unchecked<uint8_t>(RPU_RPT_HKCURR_BITS);
            pump_i_raw_  = bsr.read_unchecked<uint8_t>(RPU_RPT_HKCURR_BITS);
            tdlas_i_raw_ = bsr.read_unchecked<uint8_t>(RPU_RPT_HKCURR_BITS);
            v5v_raw_     = bsr.read_unchecked<uint8_t>(RPU_RPT_V5V_BITS);
            break;
        case 7:
            bat_t_raw_    = bsr.read_unchecked<uint8_t> (RPU_RPT_HKTEMP_BITS);
            pump_t_raw_   = bsr.read_unchecked<uint8_t> (RPU_RPT_HKTEMP_BITS);
            pcb_t_raw_    = bsr.read_unchecked<uint8_t> (RPU_RPT_HKTEMP_BITS);
            bat_v_raw_    = bsr.read_unchecked<uint16_t>(RPU_RPT_VOLT_BITS);
            heater_stat_  = bsr.read_unchecked<uint8_t> (RPU_RPT_HEATER_BITS);
            break;
    }

    return true;
}

String RPURecord::toJSON() const
{
    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{\"elapsed_s\":%u,\"alt\":%.0f,\"lat_delta\":%.5f,\"lon_delta\":%.5f,"
        "\"sats\":%u,\"gps_age_s\":%u,"
        "\"opc_d300\":%u,\"opc_d2000\":%u,"
        "\"tsen_airt\":%u,\"tsen_pres\":%u,\"tsen_ptemp\":%u,"
        "\"rs41_air_t\":%.2f,\"rs41_pres\":%.1f,\"rs41_humidity\":%.2f,\"rs41_hsensor_t\":%.2f,"
        "\"tdlas_mr_avg\":%.1f,\"tdlas_bkg\":%.2f,\"tdlas_peak\":%.1f,\"tdlas_ratio\":%.3f,"
        "\"round_robin_idx\":%u,"
        "\"opc_d500\":%u,\"opc_d700\":%u,\"opc_d1000\":%u,\"opc_d3000\":%u,\"opc_d5000\":%u,\"opc_d2500\":%u,"
        "\"rs41_hdg\":%.2f,\"bemf_v\":%.3f,"
        "\"tdlas_spec_1\":%.0f,\"tdlas_spec_2\":%.0f,\"tdlas_spec_3\":%.0f,\"tdlas_spec_4\":%.0f,"
        "\"tsen_i\":%.0f,\"opc_i\":%.0f,\"pump_i\":%.0f,\"tdlas_i\":%.0f,"
        "\"v5\":%.2f,\"bat_t\":%.0f,\"pump_t\":%.0f,\"pcb_t\":%.0f,\"bat_v\":%.2f,\"heater_stat\":%u}",
        elapsed_s_, getAlt(), getLatDelta(), getLonDelta(),
        sats_, gps_age_s_,
        opc_d300_, opc_d2000_,
        tsen_airt_raw_, tsen_pres_raw_, tsen_ptemp_raw_,
        getRs41AirT(), getRs41Pres(), getRs41Humidity(), getRs41HSensorT(),
        getTdlasMrAvg(), getTdlasBkg(), getTdlasPeak(), getTdlasRatio(),
        round_robin_idx_,
        opc_d500_, opc_d700_, opc_d1000_, opc_d3000_, opc_d5000_, opc_d2500_,
        getRs41Hdg(), getBemfV(),
        getTdlasSpec1(), getTdlasSpec2(), getTdlasSpec3(), getTdlasSpec4(),
        getTsenI(), getOpcI(), getPumpI(), getTdlasI(),
        getV5V(), getBatT(), getPumpT(), getPcbT(), getBatV(), heater_stat_);

    return String(buf);
}

