# RPURecord Design Notes

This document records the design discussion and scaling decisions behind the
`RPURecord` class in `RPUcomm.h` / `RPUcomm.cpp`.

## Goal

`RPURecord` bit-packs one `tickMeasure()` sample into a **fixed 40-byte
(320-bit) record**, matching the "Profiler TM Format 2026" spec
(`docs/Profiler TM Format 2026.xlsx`): 5000 records per profile x 40 bytes =
200,000 bytes (~195 KiB), close to the spec's 190,000-byte target. This is
`RPU_RPT_VERSION = 1`, the first released `RPURecord` wire format.

It follows the same pattern as `RPUPacket`:

- private raw fields holding a scaled/offset integer representation
- public setters that convert engineering units -> packed representation
- public getters that convert packed representation -> engineering units
- `encode()` / `decode()` using `etl::bit_stream_writer` / `bit_stream_reader`
  (big-endian, `write_unchecked` / `read_unchecked`)
- `toJSON()` for human-readable debugging

### Why this layout (and not the earlier 134-byte draft)

An earlier draft packed every value gathered in `tickMeasure()`, including
several raw `float32` TDLAS fields and RS41 housekeeping fields that the 2026
spec doesn't carry at full rate, into 134 bytes/record. At 5000 records that
would be 670,000 bytes/profile — too large. That draft was never released, so
this design supersedes it directly as `RPU_RPT_VERSION = 1`, trimming the
per-record payload to 40 bytes by:

- dropping fields the spec doesn't carry at all (RS41 `valid`, internal temp,
  module status/error, pcb supply V, lsm303 temp, pcb heater flag, frame
  count; absolute lat/lon; board ID; `pump.pwm`; `ROPC_time`; OPC alarm)
- replacing absolute GPS lat/lon with small **deltas from a session-start
  reference**
- splitting fields into a **fast group** (period = 1, present every record)
  and a **slow/round-robin group** (period = 8, one field-pair "slot" per
  record, cycling through 8 slots so each slow field is sent roughly every 8
  records)
- replacing raw `float32` TDLAS fields with provisional fixed-point scales

## Field groups

### Fast fields (period = 1, 270 bits/record)

Present in every record, in this order:

| # | Field | Setter / Getter | Encoding | Bits |
|---|---|---|---|---|
| 1 | round-robin index | `setRoundRobinIdx`/`getRoundRobinIdx` | 0–7, selects which slow-field slot follows | 4 |
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
| 13 | RS41 air temperature | `setRs41AirT`/`getRs41AirT` | `(T+100) x100`, -100.00 to 555.35 °C | 16 |
| 14 | RS41 pressure | `setRs41Pres`/`getRs41Pres` | `x10`, 0–6553.5 mb | 16 |
| 15 | RS41 RH | `setRs41Humidity`/`getRs41Humidity` | `x100`, 0–655.35 % | 16 |
| 16 | RS41 temp-of-RH | `setRs41HSensorT`/`getRs41HSensorT` | `(T+100) x100`, -100.00 to 555.35 °C | 16 |
| 17 | TDLAS VMR_ave | `setTdlasMrAvg`/`getTdlasMrAvg` | `x10`, 0–102.3 (provisional, see TDLAS section) | 10 |
| 18 | *(reserved)* | none | always 0 on encode, skipped on decode | 10 |
| 19 | TDLAS bkg | `setTdlasBkg`/`getTdlasBkg` | `x100`, 0–40.95 (provisional) | 12 |
| 20 | TDLAS peak | `setTdlasPeak`/`getTdlasPeak` | `x10`, 0–25.5 (provisional) | 8 |
| 21 | TDLAS ratio | `setTdlasRatio`/`getTdlasRatio` | `x1000`, 0–1.023 (provisional) | 10 |

13 fields x 16 bits (208) + 3 fields x 4 bits (12) + 3 fields x 10 bits (30) +
1 field x 12 bits (12) + 1 field x 8 bits (8) = **270 bits**.

### Slow / round-robin fields (period = 8, one 40-bit slot/record)

22 fields total, sent one pair (or group) at a time via the round-robin
index. Every `RPURecord` setter is called every tick (so the in-memory object
always holds the latest reading of all 22 fields), but `encode()` only
serialises the 40-bit slot selected by `round_robin_idx_`; `decode()` only
populates that same slot, leaving the other slow members at their default
(zero) values.

| Index | Slot contents | Bits | Setters / Getters |
|---|---|---|---|
| 0 | ROPC 500nm (16) + ROPC 700nm (16) + pad (8) | 40 | `setOpcD500`/`setOpcD700` |
| 1 | ROPC 1000nm (16) + ROPC 2500nm (16) + pad (8) | 40 | `setOpcD1000`/`setOpcD2500` |
| 2 | ROPC 3000nm (16) + ROPC 5000nm (16) + pad (8) | 40 | `setOpcD3000`/`setOpcD5000` |
| 3 | RS41 magnetometer X-Y (16) + pump BEMF (16) + pad (8) | 40 | `setRs41MagXY`/`setBemfV` |
| 4 | TDLAS spectra 1 (16) + TDLAS spectra 2 (16) + pad (8) | 40 | `setTdlasSpec1`/`setTdlasSpec2` |
| 5 | TDLAS spectra 3 (16) + TDLAS spectra 4 (16) + pad (8) | 40 | `setTdlasSpec3`/`setTdlasSpec4` |
| 6 | I_TSEN (8) + I_ROPC (8) + I_PUMP (8) + I_TDLAS (8) + V_5V (8) | 40 | `setTsenI`/`setOpcI`/`setPumpI`/`setTdlasI`/`setV5V` |
| 7 | T_Batt (8) + T_Pump (8) + T_PCB (8) + V_Batt (12) + Heater_stat (4) | 40 | `setBatT`/`setPumpT`/`setPcbT`/`setBatV`/`setHeaterStat` |

Six slots are two 16-bit fields + 8 bits of padding (32+8=40); the last two
slots are five fields each that sum to exactly 40 bits with no padding.
12 x 16-bit fields + 10 fields totalling 80 bits = **272 bits** across the 8
possible slots — but only one slot (40 bits) is actually transmitted per
record.

### TDLAS scaling (provisional)

`TDLASData` (`RPUTDLAS.h`) and `RPUtest.cpp` agree on the available TDLAS
fields: `mr_avg`, `bkg`, `peak`, `ratio`, `batt`, `therm_1`, `therm_2`, `indx`,
`spec_1`-`spec_4`. None of these have documented physical ranges, and the
spec's "VMR_min" field has **no corresponding source field** in the current
TDLAS firmware at all.

Decisions:

- `VMR_ave`/`bkg`/`peak`/`ratio` get provisional fixed-point scales (x10,
  x100, x10, x1000 respectively) chosen to give a plausible range with the
  spec's bit widths (10/12/8/10 bits). These are **TBD placeholders** —
  revisit once real TDLAS data ranges are characterized.
- `VMR_min` (10 bits in the spec) has no data source. Its bits are encoded as
  `RPU_RPT_RESERVED_BITS`: always written as 0 by `encode()`, read and
  discarded by `decode()`, with no setter/getter. This preserves the planned
  record length (and the spec's bit layout) for when/if a VMR_min source
  becomes available.
- `spec_1`-`spec_4` are packed as a **raw 16-bit passthrough** (`(uint16_t)`
  cast of the `float` value, clamped 0–65535) — also provisional, pending a
  real scale.
- `batt`, `therm_1`, `therm_2`, `indx` from `TDLASData` are **not** carried —
  the spec's slow-field list doesn't include them.

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

Reconstructing absolute lat/lon from `(GPSStartLat/Lon, lat_delta, lon_delta)`
is a ground-station/RATCHuTS concern — `RPUStartLat`/`RPUStartLon` must be
transmitted separately (e.g. once per profile in a header record), which is
**out of scope for `RPURecord` itself** and is a known follow-up.

### TSEN raw values

`tsenRaw.airt_raw` is a 12-bit A/D count (0–0xFFF); `tsenRaw.ptemp_raw` and
`tsenRaw.pres_raw` are 24-bit counts (0–0xFFFFFF). The spec only allows 16
bits per TSEN field, so `setTsenPres`/`setTsenPtemp` keep the **top 16 bits**
of the 24-bit count (`raw >> 8`), discarding the bottom 8 bits of precision.

## Round-robin cycling

`RPU.cpp` keeps a `RoundRobinIdx` (0–7) that is reset to 0 in `enterMeasure()`
and incremented (mod 8) after every `tickMeasure()` push. Every tick, *all*
22 slow-field setters are called with the latest readings (so the in-memory
`RPURecord` is always fully populated), but `encode()` only transmits the
40-bit slot for the current `RoundRobinIdx`. Over 8 consecutive records, all
22 slow fields are eventually transmitted once.

## Final bit layout / packet size

Total payload: 4 (version) + 270 (fast fields) + 40 (one round-robin slot) +
6 (trailing pad) = **320 bits = 40 bytes** (`RPU_RPT_BYTES`).

| Constant | Bits | Used for |
|---|---|---|
| `RPU_RPT_VER_BITS` | 4 | packet format version (`RPU_RPT_VERSION = 1`) |
| `RPU_RPT_ELAPSED_BITS` | 16 | elapsed seconds since `MeasureStartMillis` |
| `RPU_RPT_ALT_BITS` | 16 | altitude, m, raw |
| `RPU_RPT_GPS_DELTA_BITS` | 16 | `(lat\|lon - start) x50000`, signed |
| `RPU_RPT_SATS_BITS` | 4 | satellite count (0–15) |
| `RPU_RPT_GPS_AGE_BITS` | 4 | GPS fix age, s, clamped (0–15 s) |
| `RPU_RPT_OPC_BITS` | 16 | OPC bin counts, raw |
| `RPU_RPT_TSEN_BITS` | 16 | TSEN raw counts (airt: 0–4095; pres/ptemp: top 16 bits of 24-bit count) |
| `RPU_RPT_RS41_T_BITS` | 16 | `(T+100) x100` (-100.00 to 555.35 °C) |
| `RPU_RPT_RS41_P_BITS` | 16 | pressure `x10` (0–6553.5 mb) |
| `RPU_RPT_RS41_RH_BITS` | 16 | RH `x100` (0–655.35 %) |
| `RPU_RPT_TDLAS_VMR_BITS` | 10 | TDLAS VMR_ave `x10`, provisional (0–102.3) |
| `RPU_RPT_RESERVED_BITS` | 10 | reserved (spec: TDLAS VMR_min — not available from current TDLAS firmware) |
| `RPU_RPT_TDLAS_BKG_BITS` | 12 | TDLAS bkg `x100`, provisional (0–40.95) |
| `RPU_RPT_TDLAS_PEAK_BITS` | 8 | TDLAS peak `x10`, provisional (0–25.5) |
| `RPU_RPT_TDLAS_RATIO_BITS` | 10 | TDLAS ratio `x1000`, provisional (0–1.023) |
| `RPU_RPT_RR_IDX_BITS` | 4 | round-robin slot index (0–7) |
| `RPU_RPT_MAGXY_BITS` | 16 | RS41 magnetometer X-Y, counts + 1000 |
| `RPU_RPT_BEMF_BITS` | 16 | pump BEMF, V `x1000` |
| `RPU_RPT_SPEC_BITS` | 16 | TDLAS spectra, raw passthrough, provisional |
| `RPU_RPT_HKCURR_BITS` | 8 | subsystem currents, mA/4 (0–1020 mA, 4 mA res) |
| `RPU_RPT_V5V_BITS` | 8 | V `x50` (0–5.10 V, 0.02 V res) |
| `RPU_RPT_HKTEMP_BITS` | 8 | `(T+100)`, 1 °C res (-100 to 155 °C) |
| `RPU_RPT_VOLT_BITS` | 12 | battery voltage `x100` (0–40.95 V) |
| `RPU_RPT_HEATER_BITS` | 4 | heater status (bit0: battery heater on) |
| `RPU_RPT_SLOT_PAD_BITS` | 8 | padding within the two-field 40-bit slots (indices 0–5) |
| `RPU_RPT_PAD_BITS` | 6 | trailing reserved padding to reach a byte boundary |

## Open items / not yet done

- `RPUStartLat`/`RPUStartLon`/`RPUStartTime` (the GPS reference point each
  record's deltas are relative to) must be transmitted separately, e.g. once
  per profile in a header/status message — not part of `RPURecord`.
- TDLAS scales (`VMR_ave`, `bkg`, `peak`, `ratio`, `spec_1`-`4`) are
  placeholders pending real instrument range/resolution data.
- `VMR_min` has no data source and its 10 bits are always zero.
- The `-Wformat-truncation` warning GCC may emit for `RPURecord::toJSON()` is
  a known false positive — the underlying values are bounded by their bit
  widths, so the worst-case `%f`/`%.*f` buffer size GCC assumes is
  never actually reached.
