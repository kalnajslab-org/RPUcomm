# RPURecord Design Notes

This document records the design discussion and scaling decisions behind the
`RPURecord` class in `RPUcomm.h` / `RPUcomm.cpp`.

## Goal

`RPURecord` bit-packs one `tickMeasure()` sample into a **fixed 49-byte
(392-bit) record**. This is `RPU_REC_VERSION = 2`, which finalizes the TDLAS
field layout (see "TDLAS scaling" below) that `RPU_REC_VERSION = 1` carried as
provisional placeholders.

It follows the same pattern as `RPUPacket`:

- private raw fields holding a scaled/offset integer representation
- public setters that convert engineering units -> packed representation
- public getters that convert packed representation -> engineering units
- `encode()` / `decode()` using `etl::bit_stream_writer` / `bit_stream_reader`
  (big-endian, `write_unchecked` / `read_unchecked`)
- `toJSON()` for human-readable debugging

### Why this layout (and not the earlier 134-byte draft)

An earlier draft packed every value gathered in `tickMeasure()`, including
several raw `float32` TDLAS fields and RS41 housekeeping fields, into 134
bytes/record. At 5000 records that would be 670,000 bytes/profile — too large.
The current design trims the per-record payload to 48 bytes by:

- dropping fields the spec doesn't carry at all (RS41 `valid`, internal temp,
  module status/error, pcb supply V, lsm303 temp, pcb heater flag, frame
  count; absolute lat/lon; board ID; `pump.pwm`; `ROPC_time`; OPC alarm)
- replacing absolute GPS lat/lon with small **deltas from a session-start
  reference** (absolute reference transmitted once per block in a block header)
- splitting fields into a **fast group** (period = 1, present every record)
  and a **slow/round-robin group** (period = 6, one field-pair "slot" per
  record, cycling through 6 slots so each slow field is sent roughly every 6
  records)
- replacing raw `float32` TDLAS fields with fixed-point scales
- transmitting all four TDLAS cluster values as fast fields every record;
  `tdlas_cluster_idx` is an instrument data value (not a selector) transmitted alongside

## Field groups

### Fast fields (period = 1, 352 bits/record including version)

Present in every record, in this order:

| # | Field | Setter / Getter | Encoding | Bits |
|---|---|---|---|---|
| — | record format version | encoded internally | fixed value `RPU_REC_VERSION = 2` | 4 |
| 1 | round-robin index | managed internally (see "Round-robin cycling") | 0–5, selects which slow-field slot follows | 4 |
| 2 | elapsed time | `setElapsedS`/`getElapsedS` | seconds since `MeasureStartMillis`, raw uint16 (0–65535 s) | 16 |
| 3 | GPS altitude | `setAlt`/`getAlt` | meters, raw uint16 | 16 |
| 4 | GPS latitude delta | `setLatDelta`/`getLatDelta` | `(lat - GPSStartLat) x50000`, signed int16 (±0.65534°, ~2.2 m res at equator) | 16 |
| 5 | GPS longitude delta | `setLonDelta`/`getLonDelta` | `(lon - GPSStartLon) x50000`, signed int16 | 16 |
| 6 | GPS satellites | `setSats`/`getSats` | raw count, clamped 0–15 | 4 |
| 7 | GPS fix age | `setGpsAge`/`getGpsAge` | seconds, clamped 0–15 | 4 |
| 8 | ROPC 300nm | `setOpcD300`/`getOpcD300` | raw count | 16 |
| 9 | ROPC 2000nm | `setOpcD2000`/`getOpcD2000` | raw count | 16 |
| 10 | TSEN air temperature | `setTsenAirt`/`getTsenAirt` | raw 12-bit A/D count (0–0xFFF) | 16 |
| 11 | TSEN pressure | `setTsenPres`/`getTsenPres` | top 16 bits of raw 24-bit count | 16 |
| 12 | TSEN temp-of-pressure | `setTsenPtemp`/`getTsenPtemp` | top 16 bits of raw 24-bit count | 16 |
| 13 | RS41 air temperature | `setRs41AirT`/`getRs41AirT` | `(T+100) x436.9067`, -100 to +50 °C | 16 |
| 14 | RS41 pressure | `setRs41Pres`/`getRs41Pres` | `(ln(P) - 3.9120) x21525.87`, 50–1050 hPa | 16 |
| 15 | RS41 RH | `setRs41Humidity`/`getRs41Humidity` | `(RH+20) x543.1333`, -20 to +100 %RH | 16 |
| 16 | RS41 temp-of-RH | `setRs41HSensorT`/`getRs41HSensorT` | `(T+100) x436.9067`, -100 to +50 °C | 16 |
| 17 | TDLAS mixing ratio | `setTdlasMixingRatio`/`getTdlasMixingRatio` | `x100`, 0–2621.43 | 18 |
| 18 | TDLAS background | `setTdlasBackground`/`getTdlasBackground` | raw counts, 0–4095 | 12 |
| 19 | TDLAS peak | `setTdlasPeak`/`getTdlasPeak` | `x10`, 0–51.1 | 9 |
| 20 | TDLAS ratio | `setTdlasRatio`/`getTdlasRatio` | `x10`, 0–3.1 | 5 |
| 21 | TDLAS laser temperature | `setTdlasLaserTemp`/`getTdlasLaserTemp` | `x100`, 0–40.95 °C | 12 |
| 22 | TDLAS max mixing ratio | `setTdlasMrMaxRatio`/`getTdlasMrMaxRatio` | `x10`, 0–12.7 | 7 |
| 23 | TDLAS status | `setTdlasStatus`/`getTdlasStatus` | instrument status code, 0–31 | 5 |
| 24 | TDLAS cluster index | `setTdlasClusterIdx`/`getTdlasClusterIdx` | 0–15, instrument data value | 4 |
| 25 | TDLAS cluster value 1 | `setTdlasCluster1`/`getTdlasCluster1` | `x100`, 0–163.83 | 14 |
| 26 | TDLAS cluster value 2 | `setTdlasCluster2`/`getTdlasCluster2` | `x100`, 0–163.83 | 14 |
| 27 | TDLAS cluster value 3 | `setTdlasCluster3`/`getTdlasCluster3` | `x100`, 0–163.83 | 14 |
| 28 | TDLAS cluster value 4 | `setTdlasCluster4`/`getTdlasCluster4` | `x100`, 0–163.83 | 14 |

Bit tally:
- version (4) + rr_idx (4) + fields 2–16 (13 × 16 = 208) + sats/age (2 × 4 = 8)
  + mixing_ratio (18) + background (12) + peak (9) + ratio (5) + laser_temp (12)
  + mr_max_ratio (7) + status (5) + cluster_idx (4) + cluster1–4 (4 × 14 = 56)
  = **352 bits**.

Total TDLAS fast bits: 18+12+9+5+12+7+5+4+56 = **128 bits**.

### Slow / round-robin fields (period = 6, one 40-bit slot/record)

17 fields total, sent one group at a time via the round-robin index. Every
`RPURecord` setter is called every tick (so the in-memory object always holds
the latest reading of all fields), but `encode()` only serialises the 40-bit
slot selected by `round_robin_idx_`; `decode()` only populates that same slot,
leaving the other slow members at their default (zero) values.

| Index | Slot contents | Bits | Setters / Getters |
|---|---|---|---|
| 0 | ROPC 500nm (16) + ROPC 700nm (16) + pad (8) | 40 | `setOpcD500`/`setOpcD700` |
| 1 | ROPC 1000nm (16) + ROPC 2500nm (16) + pad (8) | 40 | `setOpcD1000`/`setOpcD2500` |
| 2 | ROPC 3000nm (16) + ROPC 5000nm (16) + pad (8) | 40 | `setOpcD3000`/`setOpcD5000` |
| 3 | RS41 heading (8) + pump BEMF (16) + RS41 status flags (8) + pad (8) | 40 | `setRs41Hdg`/`setBemfV`/`setRs41Status` |
| 4 | I_TSEN (8) + I_ROPC (8) + I_PUMP (8) + I_TDLAS (8) + V_5V (8) | 40 | `setTsenI`/`setOpcI`/`setPumpI`/`setTdlasI`/`setV5V` |
| 5 | T_Batt (8) + T_Pump (8) + T_PCB (8) + V_Batt (12) + Heater_stat (4) | 40 | `setBatT`/`setPumpT`/`setPcbT`/`setBatV`/`setHeaterStat` |

Slots 0–2 are two 16-bit fields + 8 bits of padding (32+8=40). Slot 3 is an
8-bit heading + 16-bit BEMF + 8-bit RS41 status flags + 8 bits of padding
(8+16+8+8=40). Slots 4–5 are five fields each that sum to exactly 40 bits with
no padding.

The RS41 status byte packs eight `RS41StatusFlags_t` boolean fields as bits
(LSB = bit 0), defined by the `RPU_REC_RS41_*` constants in `RPUcomm.h`:

| Bit | Constant | Flag |
|-----|----------|------|
| 0 | `RPU_REC_RS41_HIGH_INTERNAL_TEMP` | S.2: high internal temperature |
| 1 | `RPU_REC_RS41_REGEN_TEMP_LOW` | S.3: regen temperature low |
| 2 | `RPU_REC_RS41_PTU_FAILURE` | S.4: PTU failure |
| 3 | `RPU_REC_RS41_FLASH_FAILURE` | S.5: flash failure |
| 4 | `RPU_REC_RS41_LOW_INPUT_VOLTAGE` | E.6: low input voltage |
| 5 | `RPU_REC_RS41_NOT_CALIBRATED` | E.7: not calibrated |
| 6 | `RPU_REC_RS41_NO_PRESSURE_MODULE` | E.8: no pressure module |
| 7 | `RPU_REC_RS41_DISCONNECTED_BOOM` | E.9: disconnected boom |

### TDLAS scaling

All TDLAS fields are fast (period = 1, present in every record). The four
cluster values are sent in full every record; `tdlas_cluster_idx` is a data
value from the instrument (not a mux selector), transmitted alongside them.

| Field | Scale | Bit width | Range |
|---|---|---|---|
| mixing ratio | ×100 | 18 | 0–2621.43 |
| background | — (raw counts) | 12 | 0–4095 |
| peak | ×10 | 9 | 0–51.1 |
| ratio | ×10 | 5 | 0–3.1 |
| laser temperature | ×100 | 12 | 0–40.95 °C |
| max mixing ratio | ×10 | 7 | 0–12.7 |
| status | — | 5 | 0–31 |
| cluster idx | — | 4 | 0–15 |
| cluster 1–4 | ×100 | 14 each | 0–163.83 |

These scales are the finalized instrument spec ranges (as of `RPU_REC_VERSION = 2`),
superseding the provisional placeholders used in version 1.

### GPS delta encoding

The spec encodes GPS position as small deltas from a per-profile reference
point rather than absolute lat/lon, to fit in 16 bits each:

- `GPSStartLat`/`GPSStartLon` (`double`) and `MeasureStartMillis` (`uint32_t`)
  are captured once, in `tickMeasure()`, the first time `profiler_gps.location.isValid()`
  is true after `enterMeasure()` resets `GPSStartCaptured = false`.
- `elapsed_s = (millis() - MeasureStartMillis) / 1000`.
- `lat_delta = (profiler_gps.location.lat() - GPSStartLat) x50000`,
  `lon_delta` likewise — both signed int16, range ±0.65534° (~±72 km lat,
  less in lon depending on latitude), 1/50000° (~2.2 m) resolution.

Reconstructing absolute lat/lon from per-record deltas requires the reference
point `(GPSStartLat, GPSStartLon)`, which is transmitted once per block in the
block header (see "Block header" below).

### TSEN raw values

`tsenRaw.airt_raw` is a 12-bit A/D count (0–0xFFF); `tsenRaw.ptemp_raw` and
`tsenRaw.pres_raw` are 24-bit counts (0–0xFFFFFF). The spec only allows 16
bits per TSEN field, so `setTsenPres`/`setTsenPtemp` keep the **top 16 bits**
of the 24-bit count (`raw >> 8`), discarding the bottom 8 bits of precision.

## Block header

A block header is prepended to every group of `RPURecord`s before transmission.
It contains the GPS reference data needed to reconstruct absolute position and
UTC time from the per-record deltas and elapsed seconds.

**Size:** `RPU_BLOCK_HDR_BYTES = 12` bytes (96 bits).

**Wire format** (big-endian, in order):

| Field | Type | Bits | Notes |
|---|---|---|---|
| `epoch_time` | uint32 | 32 | Unix UTC epoch seconds, captured at GPS-start-capture via `mktime()` |
| `gps_lat` | int32 | 32 | Latitude × 1 000 000, i.e. degrees × 1e6, signed |
| `gps_lon` | int32 | 32 | Longitude × 1 000 000, i.e. degrees × 1e6, signed |

These three values are stored as non-bit-packed metadata on `RPURecord`
(`epoch_time_`, `gps_lat_`, `gps_lon_`) and are set once at GPS-start-capture
in `RPU.cpp`:

```cpp
rpu_record.setGpsLat(GPSStartLat);
rpu_record.setGpsLon(GPSStartLon);
struct tm t = {};
t.tm_year = profiler_gps.date.year() - 1900;
t.tm_mon  = profiler_gps.date.month() - 1;
t.tm_mday = profiler_gps.date.day();
t.tm_hour = profiler_gps.time.hour();
t.tm_min  = profiler_gps.time.minute();
t.tm_sec  = profiler_gps.time.second();
rpu_record.setEpochTime((uint32_t)mktime(&t));
```

On Teensy (newlib, no OS timezone), `mktime()` defaults to UTC — no timezone
adjustment is needed. `sendRPURecords()` calls `encodeBlockHeader()` once before
transmitting the batch of records, prepending 12 bytes to the `tm_buf`.

## Round-robin cycling

The round-robin slot index (0–5) is managed internally by `RPURecord` via a
shared rotation counter — callers never see or set its value directly:

- `RPURecord::resetRotation()` resets the rotation to slot 0. `RPU.cpp` calls
  this once per MEASURE session, in `enterMeasure()`.
- Each `RPURecord` constructor captures the current rotation slot at
  construction time.
- `RPURecord::advanceRotation()` advances the rotation (mod 6) to the next
  slot. `RPU.cpp` calls this once per `tickMeasure()`, after the record has
  been encoded/pushed.

Every tick, *all* slow-field setters are called with the latest readings (so
the in-memory `RPURecord` is always fully populated), but `encode()` only
transmits the 40-bit slot for the record's captured round-robin index. Over 6
consecutive records, all 17 slow fields are eventually transmitted once.

## Final bit layout / packet size

Total per-record payload: 352 (fast, including version) + 40 (one round-robin
slot) = **392 bits = 49 bytes** (`RPU_RECORD_BYTES`), no padding needed.

| Constant | Bits | Used for |
|---|---|---|
| `RPU_REC_VER_BITS` | 4 | record format version (`RPU_REC_VERSION = 2`) |
| `RPU_REC_RR_IDX_BITS` | 4 | round-robin slot index (0–5) |
| `RPU_REC_ELAPSED_BITS` | 16 | elapsed seconds since `MeasureStartMillis` |
| `RPU_REC_ALT_BITS` | 16 | altitude, m, raw |
| `RPU_REC_GPS_DELTA_BITS` | 16 | `(lat\|lon - start) x50000`, signed |
| `RPU_REC_SATS_BITS` | 4 | satellite count (0–15) |
| `RPU_REC_GPS_AGE_BITS` | 4 | GPS fix age, s, clamped (0–15 s) |
| `RPU_REC_OPC_BITS` | 16 | OPC bin counts, raw |
| `RPU_REC_TSEN_BITS` | 16 | TSEN raw counts (airt: 0–4095; pres/ptemp: top 16 bits of 24-bit count) |
| `RPU_REC_RS41_T_BITS` | 16 | `(T+100) x436.9067` (-100 to +50 °C) |
| `RPU_REC_RS41_P_BITS` | 16 | `(ln(P) - 3.9120) x21525.87` (50–1050 hPa) |
| `RPU_REC_RS41_RH_BITS` | 16 | `(RH+20) x543.1333` (-20 to +100 %RH) |
| `RPU_REC_TDLAS_MIXING_RATIO_BITS` | 18 | TDLAS mixing ratio `x100` (0–2621.43) |
| `RPU_REC_TDLAS_BACKGROUND_BITS` | 12 | TDLAS background, raw counts (0–4095) |
| `RPU_REC_TDLAS_PEAK_BITS` | 9 | TDLAS peak `x10` (0–51.1) |
| `RPU_REC_TDLAS_RATIO_BITS` | 5 | TDLAS ratio `x10` (0–3.1) |
| `RPU_REC_TDLAS_LASER_TEMP_BITS` | 12 | TDLAS laser temperature `x100` (0–40.95 °C) |
| `RPU_REC_TDLAS_MR_MAX_RATIO_BITS` | 7 | TDLAS max mixing ratio `x10` (0–12.7) |
| `RPU_REC_TDLAS_STATUS_BITS` | 5 | TDLAS instrument status code (0–31) |
| `RPU_REC_TDLAS_CLUSTER_IDX_BITS` | 4 | TDLAS cluster index (0–15, instrument data value) |
| `RPU_REC_TDLAS_CLUSTER_BITS` | 14 | TDLAS cluster value `x100` per channel (0–163.83) |
| `RPU_REC_HDG_BITS` | 8 | RS41 heading, `x256/360` (0–255, ~1.41° res) |
| `RPU_REC_BEMF_BITS` | 16 | pump BEMF, V `x1000` |
| `RPU_REC_RS41_STATUS_BITS` | 8 | RS41 status flags byte (8 flags, see slot 3 table) |
| `RPU_REC_HKCURR_BITS` | 8 | subsystem currents, mA/4 (0–1020 mA, 4 mA res) |
| `RPU_REC_V5V_BITS` | 8 | V `x50` (0–5.10 V, 0.02 V res) |
| `RPU_REC_HKTEMP_BITS` | 8 | `(T+100)`, 1 °C res (-100 to 155 °C) |
| `RPU_REC_VOLT_BITS` | 12 | battery voltage `x100` (0–40.95 V) |
| `RPU_REC_HEATER_BITS` | 4 | heater status (bit0: battery heater on) |
| `RPU_REC_SLOT_PAD_BITS` | 8 | padding within the two-field 40-bit slots (indices 0–3) |
| `RPU_RECORD_BYTES` | — | 49 bytes = (352 fast + 40 slow) / 8 |
| `RPU_BLOCK_HDR_BYTES` | — | 12 bytes = epoch_time (uint32) + gps_lat (int32) + gps_lon (int32) |

## Open items / not yet done

- The `-Wformat-truncation` warning GCC may emit for `RPURecord::toJSON()` is
  a known false positive — the underlying values are bounded by their bit
  widths, so the worst-case `%f`/`%.*f` buffer size GCC assumes is
  never actually reached.

## Version history

- **v1** (`RPU_REC_VERSION = 1`): first released wire format, 48 bytes/record.
  TDLAS fields (`VMR_ave`, `bkg`, `peak`, `ratio`, `max_vmr`, `laser_t`, `idx`,
  `spec1`–`4`) used provisional scales/bit widths pending real instrument data.
- **v2** (`RPU_REC_VERSION = 2`): finalized TDLAS field set and scaling per the
  instrument's actual output spec — renamed/regrouped fields (`mixing_ratio`,
  `background`, `peak`, `ratio`, `laser_temp`, `mr_max_ratio`, `status`,
  `cluster_idx`, `cluster_1`–`4`) and widened the TDLAS fast-field block from
  120 to 128 bits, growing the record from 48 to 49 bytes.
