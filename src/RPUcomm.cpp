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
    etl::bit_stream_writer bsw(buf, RPU_PKT_BYTES, etl::endian::big);

    bsw.write_unchecked<uint8_t> (RPU_PKT_VERSION,  RPU_PKT_VER_BITS);
    bsw.write_unchecked<uint16_t>(board_id_,        RPU_PKT_ID_BITS);
    bsw.write_unchecked<uint8_t> (state_,           RPU_PKT_STATE_BITS);
    bsw.write_unchecked<uint8_t> (wdt_count_,       RPU_PKT_WDT_BITS);
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

    etl::bit_stream_reader bsr(buf, RPU_PKT_BYTES, etl::endian::big);

    bsr.read_unchecked<uint8_t>(RPU_PKT_VER_BITS);  // packet version, not stored

    board_id_     = bsr.read_unchecked<uint16_t>(RPU_PKT_ID_BITS);
    state_        = bsr.read_unchecked<uint8_t> (RPU_PKT_STATE_BITS);
    wdt_count_    = bsr.read_unchecked<uint8_t> (RPU_PKT_WDT_BITS);
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
        "{\"id\":\"%04X\",\"ver\":\"%s\",\"state\":\"%s\",\"wdt_n\":%u,"
        "\"vin\":%.1f,\"v5\":%.1f,\"bat_v\":%.1f,\"bat_duty\":%u,\"chg_i\":%.1f,"
        "\"bat_t\":%.1f,\"pcb_t\":%.1f,"
        "\"pump_i\":%.0f,\"opc_i\":%.0f,\"tsen_i\":%.0f,\"tdlas_i\":%.0f,\"heater_i\":%.0f,"
        "\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"sats\":%u,"
        "\"date\":%lu,\"time\":%lu}",
        board_id_, ver_, state_str, wdt_count_,
        getVin(), getV5V(), getBatV(), heater_duty_, getChgI(),
        getBatT(), getPcbT(),
        getPumpI(), getOpcI(), getTsenI(), getTdlasI(), getHeaterI(),
        getLat(), getLon(), getAlt(), sats_,
        (unsigned long)gps_date_, (unsigned long)gps_time_);

    return String(buf);
}

// ---------------------------------------------------------------------------
// RPUReport
// ---------------------------------------------------------------------------

namespace {
    // Pack/unpack a float as its raw IEEE-754 bit pattern (full precision).
    void writeFloat32(etl::bit_stream_writer& bsw, float value)
    {
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        bsw.write_unchecked<uint32_t>(bits, RPU_RPT_F32_BITS);
    }

    float readFloat32(etl::bit_stream_reader& bsr)
    {
        uint32_t bits = bsr.read_unchecked<uint32_t>(RPU_RPT_F32_BITS);
        float value;
        memcpy(&value, &bits, sizeof(value));
        return value;
    }
}

void RPUReport::setBoardId(uint16_t id)        { board_id_   = id; }
void RPUReport::setElapsedMs(uint32_t ms)      { elapsed_ms_ = ms; }
void RPUReport::setBatV(float volts)           { bat_v_raw_    = (uint16_t)constrain((int)(volts * 100.0f), 0, 4095); }
void RPUReport::setVin(float volts)            { vin_raw_      = (uint16_t)constrain((int)(volts * 100.0f), 0, 4095); }
void RPUReport::setChargeI(float amps)         { charge_i_raw_ = (uint16_t)constrain((int)(amps * 1000.0f), 0, 8191); }
void RPUReport::setV5V(float volts)            { v5v_raw_      = (uint16_t)constrain((int)(volts * 100.0f), 0, 4095); }
void RPUReport::setPumpI(float milliamps)      { pump_i_raw_   = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUReport::setOpcI(float milliamps)       { opc_i_raw_    = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUReport::setTsenI(float milliamps)      { tsen_i_raw_   = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUReport::setTdlasI(float milliamps)     { tdlas_i_raw_  = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUReport::setHeaterI(float milliamps)    { heater_i_raw_ = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUReport::setBemfV(float volts)          { bemf_v_raw_   = (uint16_t)constrain((int)(volts * 100.0f), 0, 4095); }
void RPUReport::setPumpPwm(uint8_t pwm)        { pump_pwm_     = pwm; }
void RPUReport::setLat(double degrees)         { lat_raw_      = (uint32_t)constrain((int)((degrees + 90.0)  * 10000.0), 0, 1800000); }
void RPUReport::setLon(double degrees)         { lon_raw_      = (uint32_t)constrain((int)((degrees + 180.0) * 10000.0), 0, 3600000); }
void RPUReport::setAlt(float meters)           { alt_raw_      = (uint16_t)constrain((int)meters, 0, 65535); }
void RPUReport::setSats(uint8_t count)         { sats_         = (uint8_t)constrain((int)count, 0, 31); }
void RPUReport::setGpsDate(uint32_t date)      { gps_date_     = (uint32_t)constrain((int)date, 0, 524287); }
void RPUReport::setGpsTime(uint32_t time)      { gps_time_     = (uint32_t)constrain((int)time, 0, 33554431); }
void RPUReport::setGpsAge(uint32_t seconds)    { gps_age_s_    = (uint32_t)constrain((int)seconds, 0, 255); }
void RPUReport::setPcbT(float celsius)         { pcb_t_raw_    = (uint16_t)constrain((int)((celsius + 100.0f) * 10.0f), 0, 4095); }
void RPUReport::setPumpT(float celsius)        { pump_t_raw_   = (uint16_t)constrain((int)((celsius + 100.0f) * 10.0f), 0, 4095); }
void RPUReport::setBatT(float celsius)         { bat_t_raw_    = (uint16_t)constrain((int)((celsius + 100.0f) * 10.0f), 0, 4095); }
void RPUReport::setOpcTime(uint32_t ms)        { opc_time_     = ms; }
void RPUReport::setOpcD300(uint16_t count)     { opc_d300_     = count; }
void RPUReport::setOpcD500(uint16_t count)     { opc_d500_     = count; }
void RPUReport::setOpcD700(uint16_t count)     { opc_d700_     = count; }
void RPUReport::setOpcD1000(uint16_t count)    { opc_d1000_    = count; }
void RPUReport::setOpcD2000(uint16_t count)    { opc_d2000_    = count; }
void RPUReport::setOpcD2500(uint16_t count)    { opc_d2500_    = count; }
void RPUReport::setOpcD3000(uint16_t count)    { opc_d3000_    = count; }
void RPUReport::setOpcD5000(uint16_t count)    { opc_d5000_    = count; }
void RPUReport::setOpcAlarm(uint8_t alarm)     { opc_alarm_    = alarm; }
void RPUReport::setTsenAirt(uint16_t raw)      { tsen_airt_raw_  = (uint16_t)constrain((int)raw, 0, 0xFFF); }
void RPUReport::setTsenPtemp(uint32_t raw)     { tsen_ptemp_raw_ = (uint32_t)constrain((long)raw, 0L, 0xFFFFFFL); }
void RPUReport::setTsenPres(uint32_t raw)      { tsen_pres_raw_  = (uint32_t)constrain((long)raw, 0L, 0xFFFFFFL); }
void RPUReport::setTdlasMrAvg(float value)     { tdlas_mr_avg_  = value; }
void RPUReport::setTdlasBkg(float value)       { tdlas_bkg_     = value; }
void RPUReport::setTdlasPeak(float value)      { tdlas_peak_    = value; }
void RPUReport::setTdlasRatio(float value)     { tdlas_ratio_   = value; }
void RPUReport::setTdlasBatt(float volts)      { tdlas_batt_    = volts; }
void RPUReport::setTdlasTherm1(float celsius)  { tdlas_therm_1_ = celsius; }
void RPUReport::setTdlasTherm2(float celsius)  { tdlas_therm_2_ = celsius; }
void RPUReport::setTdlasIndx(int8_t indx)      { tdlas_indx_    = indx; }
void RPUReport::setTdlasSpec1(float value)     { tdlas_spec_1_  = value; }
void RPUReport::setTdlasSpec2(float value)     { tdlas_spec_2_  = value; }
void RPUReport::setTdlasSpec3(float value)     { tdlas_spec_3_  = value; }
void RPUReport::setTdlasSpec4(float value)     { tdlas_spec_4_  = value; }
void RPUReport::setRs41Valid(bool valid)          { rs41_valid_ = valid; }
void RPUReport::setRs41FrameCount(uint32_t count) { rs41_frame_count_ = count; }
void RPUReport::setRs41AirT(float celsius)        { rs41_air_t_raw_        = (uint16_t)constrain((int)((celsius + 100.0f) * 10.0f), 0, 4095); }
void RPUReport::setRs41Humidity(float percent)    { rs41_humidity_raw_     = (uint16_t)constrain((int)(percent * 10.0f), 0, 1023); }
void RPUReport::setRs41HSensorT(float celsius)    { rs41_hsensor_t_raw_    = (uint16_t)constrain((int)((celsius + 100.0f) * 10.0f), 0, 4095); }
void RPUReport::setRs41Pres(float millibar)       { rs41_pres_raw_         = (uint32_t)constrain((int)(millibar * 100.0f), 0, 131071); }
void RPUReport::setRs41InternalT(float celsius)   { rs41_internal_t_raw_   = (uint16_t)constrain((int)((celsius + 100.0f) * 10.0f), 0, 4095); }
void RPUReport::setRs41ModuleStatus(uint8_t status) { rs41_module_status_  = status; }
void RPUReport::setRs41ModuleError(uint8_t error)   { rs41_module_error_   = error; }
void RPUReport::setRs41PcbSupplyV(float volts)    { rs41_pcb_supply_v_raw_ = (uint16_t)constrain((int)(volts * 100.0f), 0, 4095); }
void RPUReport::setRs41Lsm303T(float celsius)     { rs41_lsm303_t_raw_     = (uint16_t)constrain((int)((celsius + 100.0f) * 10.0f), 0, 4095); }
void RPUReport::setRs41PcbHeaterOn(bool on)       { rs41_pcb_heater_on_    = on; }
void RPUReport::setRs41MagXY(int32_t counts)      { rs41_mag_xy_           = (uint8_t)(((float)constrain(counts, -1000, 1000) + 1000.0f) / 2000.0f * 255.0f); }

bool RPUReport::encode(uint8_t * buf, size_t buf_size) const
{
    if (buf_size < RPU_RPT_BYTES) return false;

    memset(buf, 0, RPU_RPT_BYTES);
    etl::bit_stream_writer bsw(buf, RPU_RPT_BYTES, etl::endian::big);

    bsw.write_unchecked<uint8_t> (RPU_RPT_VERSION,  RPU_RPT_VER_BITS);

    bsw.write_unchecked<uint16_t>(board_id_,        RPU_RPT_ID_BITS);
    bsw.write_unchecked<uint32_t>(elapsed_ms_,      RPU_RPT_TIME_BITS);
    bsw.write_unchecked<uint16_t>(bat_v_raw_,       RPU_RPT_VOLT_BITS);
    bsw.write_unchecked<uint16_t>(vin_raw_,         RPU_RPT_VOLT_BITS);
    bsw.write_unchecked<uint16_t>(charge_i_raw_,    RPU_RPT_CHGI_BITS);
    bsw.write_unchecked<uint16_t>(v5v_raw_,         RPU_RPT_VOLT_BITS);
    bsw.write_unchecked<uint16_t>(pump_i_raw_,      RPU_RPT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(opc_i_raw_,       RPU_RPT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(tsen_i_raw_,      RPU_RPT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(tdlas_i_raw_,     RPU_RPT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(heater_i_raw_,    RPU_RPT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(bemf_v_raw_,      RPU_RPT_VOLT_BITS);
    bsw.write_unchecked<uint8_t> (pump_pwm_,        RPU_RPT_PWM_BITS);

    bsw.write_unchecked<uint32_t>(lat_raw_,         RPU_RPT_LAT_BITS);
    bsw.write_unchecked<uint32_t>(lon_raw_,         RPU_RPT_LON_BITS);
    bsw.write_unchecked<uint16_t>(alt_raw_,         RPU_RPT_ALT_BITS);
    bsw.write_unchecked<uint8_t> (sats_,            RPU_RPT_SATS_BITS);
    bsw.write_unchecked<uint32_t>(gps_date_,        RPU_RPT_GPS_DATE_BITS);
    bsw.write_unchecked<uint32_t>(gps_time_,        RPU_RPT_GPS_TIME_BITS);
    bsw.write_unchecked<uint32_t>(gps_age_s_,       RPU_RPT_GPS_AGE_BITS);

    bsw.write_unchecked<uint16_t>(pcb_t_raw_,       RPU_RPT_TEMP_BITS);
    bsw.write_unchecked<uint16_t>(pump_t_raw_,      RPU_RPT_TEMP_BITS);
    bsw.write_unchecked<uint16_t>(bat_t_raw_,       RPU_RPT_TEMP_BITS);

    bsw.write_unchecked<uint32_t>(opc_time_,        RPU_RPT_TIME_BITS);
    bsw.write_unchecked<uint16_t>(opc_d300_,        RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(opc_d500_,        RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(opc_d700_,        RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(opc_d1000_,       RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(opc_d2000_,       RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(opc_d2500_,       RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(opc_d3000_,       RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint16_t>(opc_d5000_,       RPU_RPT_OPC_BITS);
    bsw.write_unchecked<uint8_t> (opc_alarm_,       RPU_RPT_ALARM_BITS);

    bsw.write_unchecked<uint16_t>(tsen_airt_raw_,   RPU_RPT_TSEN_AIRT_BITS);
    bsw.write_unchecked<uint32_t>(tsen_ptemp_raw_,  RPU_RPT_TSEN_PTEMP_BITS);
    bsw.write_unchecked<uint32_t>(tsen_pres_raw_,   RPU_RPT_TSEN_PRES_BITS);

    writeFloat32(bsw, tdlas_mr_avg_);
    writeFloat32(bsw, tdlas_bkg_);
    writeFloat32(bsw, tdlas_peak_);
    writeFloat32(bsw, tdlas_ratio_);
    writeFloat32(bsw, tdlas_batt_);
    writeFloat32(bsw, tdlas_therm_1_);
    writeFloat32(bsw, tdlas_therm_2_);
    bsw.write_unchecked<int8_t>(tdlas_indx_, RPU_RPT_INDX_BITS);
    writeFloat32(bsw, tdlas_spec_1_);
    writeFloat32(bsw, tdlas_spec_2_);
    writeFloat32(bsw, tdlas_spec_3_);
    writeFloat32(bsw, tdlas_spec_4_);

    bsw.write_unchecked(rs41_valid_);
    bsw.write_unchecked<uint32_t>(rs41_frame_count_,      RPU_RPT_TIME_BITS);
    bsw.write_unchecked<uint16_t>(rs41_air_t_raw_,        RPU_RPT_TEMP_BITS);
    bsw.write_unchecked<uint16_t>(rs41_humidity_raw_,     RPU_RPT_HUM_BITS);
    bsw.write_unchecked<uint16_t>(rs41_hsensor_t_raw_,    RPU_RPT_TEMP_BITS);
    bsw.write_unchecked<uint32_t>(rs41_pres_raw_,         RPU_RPT_PRES_BITS);
    bsw.write_unchecked<uint16_t>(rs41_internal_t_raw_,   RPU_RPT_TEMP_BITS);
    bsw.write_unchecked<uint8_t> (rs41_module_status_,    RPU_RPT_STATUS_BITS);
    bsw.write_unchecked<uint8_t> (rs41_module_error_,     RPU_RPT_STATUS_BITS);
    bsw.write_unchecked<uint16_t>(rs41_pcb_supply_v_raw_, RPU_RPT_VOLT_BITS);
    bsw.write_unchecked<uint16_t>(rs41_lsm303_t_raw_,     RPU_RPT_TEMP_BITS);
    bsw.write_unchecked(rs41_pcb_heater_on_);
    bsw.write_unchecked<uint8_t> (rs41_mag_xy_,           RPU_RPT_MAGXY_BITS);

    return true;
}

bool RPUReport::decode(const uint8_t * buf, size_t buf_size)
{
    if (buf_size < RPU_RPT_BYTES) return false;

    etl::bit_stream_reader bsr(buf, RPU_RPT_BYTES, etl::endian::big);

    bsr.read_unchecked<uint8_t>(RPU_RPT_VER_BITS);  // packet version, not stored

    board_id_     = bsr.read_unchecked<uint16_t>(RPU_RPT_ID_BITS);
    elapsed_ms_   = bsr.read_unchecked<uint32_t>(RPU_RPT_TIME_BITS);
    bat_v_raw_    = bsr.read_unchecked<uint16_t>(RPU_RPT_VOLT_BITS);
    vin_raw_      = bsr.read_unchecked<uint16_t>(RPU_RPT_VOLT_BITS);
    charge_i_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_CHGI_BITS);
    v5v_raw_      = bsr.read_unchecked<uint16_t>(RPU_RPT_VOLT_BITS);
    pump_i_raw_   = bsr.read_unchecked<uint16_t>(RPU_RPT_CURR_BITS);
    opc_i_raw_    = bsr.read_unchecked<uint16_t>(RPU_RPT_CURR_BITS);
    tsen_i_raw_   = bsr.read_unchecked<uint16_t>(RPU_RPT_CURR_BITS);
    tdlas_i_raw_  = bsr.read_unchecked<uint16_t>(RPU_RPT_CURR_BITS);
    heater_i_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_CURR_BITS);
    bemf_v_raw_   = bsr.read_unchecked<uint16_t>(RPU_RPT_VOLT_BITS);
    pump_pwm_     = bsr.read_unchecked<uint8_t> (RPU_RPT_PWM_BITS);

    lat_raw_      = bsr.read_unchecked<uint32_t>(RPU_RPT_LAT_BITS);
    lon_raw_      = bsr.read_unchecked<uint32_t>(RPU_RPT_LON_BITS);
    alt_raw_      = bsr.read_unchecked<uint16_t>(RPU_RPT_ALT_BITS);
    sats_         = bsr.read_unchecked<uint8_t> (RPU_RPT_SATS_BITS);
    gps_date_     = bsr.read_unchecked<uint32_t>(RPU_RPT_GPS_DATE_BITS);
    gps_time_     = bsr.read_unchecked<uint32_t>(RPU_RPT_GPS_TIME_BITS);
    gps_age_s_    = bsr.read_unchecked<uint32_t>(RPU_RPT_GPS_AGE_BITS);

    pcb_t_raw_    = bsr.read_unchecked<uint16_t>(RPU_RPT_TEMP_BITS);
    pump_t_raw_   = bsr.read_unchecked<uint16_t>(RPU_RPT_TEMP_BITS);
    bat_t_raw_    = bsr.read_unchecked<uint16_t>(RPU_RPT_TEMP_BITS);

    opc_time_     = bsr.read_unchecked<uint32_t>(RPU_RPT_TIME_BITS);
    opc_d300_     = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_d500_     = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_d700_     = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_d1000_    = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_d2000_    = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_d2500_    = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_d3000_    = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_d5000_    = bsr.read_unchecked<uint16_t>(RPU_RPT_OPC_BITS);
    opc_alarm_    = bsr.read_unchecked<uint8_t> (RPU_RPT_ALARM_BITS);

    tsen_airt_raw_  = bsr.read_unchecked<uint16_t>(RPU_RPT_TSEN_AIRT_BITS);
    tsen_ptemp_raw_ = bsr.read_unchecked<uint32_t>(RPU_RPT_TSEN_PTEMP_BITS);
    tsen_pres_raw_  = bsr.read_unchecked<uint32_t>(RPU_RPT_TSEN_PRES_BITS);

    tdlas_mr_avg_  = readFloat32(bsr);
    tdlas_bkg_     = readFloat32(bsr);
    tdlas_peak_    = readFloat32(bsr);
    tdlas_ratio_   = readFloat32(bsr);
    tdlas_batt_    = readFloat32(bsr);
    tdlas_therm_1_ = readFloat32(bsr);
    tdlas_therm_2_ = readFloat32(bsr);
    tdlas_indx_    = bsr.read_unchecked<int8_t>(RPU_RPT_INDX_BITS);
    tdlas_spec_1_  = readFloat32(bsr);
    tdlas_spec_2_  = readFloat32(bsr);
    tdlas_spec_3_  = readFloat32(bsr);
    tdlas_spec_4_  = readFloat32(bsr);

    rs41_valid_            = bsr.read_unchecked<bool>();
    rs41_frame_count_      = bsr.read_unchecked<uint32_t>(RPU_RPT_TIME_BITS);
    rs41_air_t_raw_        = bsr.read_unchecked<uint16_t>(RPU_RPT_TEMP_BITS);
    rs41_humidity_raw_     = bsr.read_unchecked<uint16_t>(RPU_RPT_HUM_BITS);
    rs41_hsensor_t_raw_    = bsr.read_unchecked<uint16_t>(RPU_RPT_TEMP_BITS);
    rs41_pres_raw_         = bsr.read_unchecked<uint32_t>(RPU_RPT_PRES_BITS);
    rs41_internal_t_raw_   = bsr.read_unchecked<uint16_t>(RPU_RPT_TEMP_BITS);
    rs41_module_status_    = bsr.read_unchecked<uint8_t> (RPU_RPT_STATUS_BITS);
    rs41_module_error_     = bsr.read_unchecked<uint8_t> (RPU_RPT_STATUS_BITS);
    rs41_pcb_supply_v_raw_ = bsr.read_unchecked<uint16_t>(RPU_RPT_VOLT_BITS);
    rs41_lsm303_t_raw_     = bsr.read_unchecked<uint16_t>(RPU_RPT_TEMP_BITS);
    rs41_pcb_heater_on_    = bsr.read_unchecked<bool>();
    rs41_mag_xy_           = bsr.read_unchecked<uint8_t> (RPU_RPT_MAGXY_BITS);

    return true;
}

String RPUReport::toJSON() const
{
    char buf[1400];
    snprintf(buf, sizeof(buf),
        "{\"id\":\"%04X\",\"elapsed_ms\":%lu,"
        "\"bat_v\":%.2f,\"vin\":%.2f,\"charge_i\":%.3f,\"v5\":%.2f,"
        "\"pump_i\":%.0f,\"opc_i\":%.0f,\"tsen_i\":%.0f,\"tdlas_i\":%.0f,\"heater_i\":%.0f,"
        "\"bemf_v\":%.2f,\"pump_pwm\":%u,"
        "\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"sats\":%u,"
        "\"gps_date\":%lu,\"gps_time\":%lu,\"gps_age_s\":%lu,"
        "\"pcb_t\":%.1f,\"pump_t\":%.1f,\"bat_t\":%.1f,"
        "\"opc_time\":%lu,"
        "\"opc_d300\":%u,\"opc_d500\":%u,\"opc_d700\":%u,\"opc_d1000\":%u,"
        "\"opc_d2000\":%u,\"opc_d2500\":%u,\"opc_d3000\":%u,\"opc_d5000\":%u,\"opc_alarm\":%u,"
        "\"tsen_airt\":%u,\"tsen_ptemp\":%lu,\"tsen_pres\":%lu,"
        "\"tdlas_mr_avg\":%.4f,\"tdlas_bkg\":%.4f,\"tdlas_peak\":%.4f,\"tdlas_ratio\":%.6f,"
        "\"tdlas_batt\":%.3f,\"tdlas_therm_1\":%.2f,\"tdlas_therm_2\":%.2f,\"tdlas_indx\":%d,"
        "\"tdlas_spec_1\":%.4f,\"tdlas_spec_2\":%.4f,\"tdlas_spec_3\":%.4f,\"tdlas_spec_4\":%.4f,"
        "\"rs41_valid\":%s,"
        "\"rs41_frame_count\":%lu,\"rs41_air_t\":%.1f,\"rs41_humidity\":%.1f,\"rs41_hsensor_t\":%.1f,"
        "\"rs41_pres\":%.2f,\"rs41_internal_t\":%.1f,"
        "\"rs41_module_status\":%u,\"rs41_module_error\":%u,"
        "\"rs41_pcb_supply_v\":%.2f,\"rs41_lsm303_t\":%.1f,\"rs41_pcb_heater_on\":%s,"
        "\"rs41_mag_xy\":%ld}",
        board_id_, (unsigned long)elapsed_ms_,
        getBatV(), getVin(), getChargeI(), getV5V(),
        getPumpI(), getOpcI(), getTsenI(), getTdlasI(), getHeaterI(),
        getBemfV(), pump_pwm_,
        getLat(), getLon(), getAlt(), sats_,
        (unsigned long)gps_date_, (unsigned long)gps_time_, (unsigned long)gps_age_s_,
        getPcbT(), getPumpT(), getBatT(),
        (unsigned long)opc_time_,
        opc_d300_, opc_d500_, opc_d700_, opc_d1000_,
        opc_d2000_, opc_d2500_, opc_d3000_, opc_d5000_, opc_alarm_,
        tsen_airt_raw_, (unsigned long)tsen_ptemp_raw_, (unsigned long)tsen_pres_raw_,
        getTdlasMrAvg(), getTdlasBkg(), getTdlasPeak(), getTdlasRatio(),
        getTdlasBatt(), getTdlasTherm1(), getTdlasTherm2(), tdlas_indx_,
        getTdlasSpec1(), getTdlasSpec2(), getTdlasSpec3(), getTdlasSpec4(),
        rs41_valid_ ? "true" : "false",
        (unsigned long)rs41_frame_count_, getRs41AirT(), getRs41Humidity(), getRs41HSensorT(),
        getRs41Pres(), getRs41InternalT(),
        rs41_module_status_, rs41_module_error_,
        getRs41PcbSupplyV(), getRs41Lsm303T(), rs41_pcb_heater_on_ ? "true" : "false",
        (long)getRs41MagXY());

    return String(buf);
}

