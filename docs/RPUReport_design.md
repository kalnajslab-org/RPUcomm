# RPUReport Design Notes

This document records the design discussion and scaling decisions behind the
`RPUReport` class in `RPUcomm.h` / `RPUcomm.cpp`.

## Goal

`RPUReport` bit-packs every value measured once per `tickMeasure()` iteration
in `RPU.cpp`, in the same order they are gathered there. It follows the same
pattern as the existing `RPUPacket` class:

- private raw fields holding a scaled/offset integer representation
- public setters that convert engineering units -> packed representation
- public getters that convert packed representation -> engineering units
- `encode()` / `decode()` using `etl::bit_stream_writer` / `bit_stream_reader`
  (big-endian, `write_unchecked` / `read_unchecked`)
- `toJSON()` for human-readable debugging

### How RPUReport differs from RPUPacket

`RPUPacket` is the LoRa status packet sent RPU -> RATCHuTS, and is hard
limited to 250 bytes (currently 40 bytes / 320 bits). `RPUReport` is sent in
bulk over the docking serial connector (`DOCK_SERIAL`, the same physical link
`rpucomm` already uses), so there is no comparable size limit. The brief was:

> Compact like RPUPacket, but this does not have a LoRa length limit because
> it will be transmitted in bulk over DOCK_SERIAL.

"Compact" here was interpreted as *bit-packed with no byte-alignment padding*
(as `RPUPacket` already does), not as *aggressively lossy*. Where a field's
physical range and required resolution are well understood, a fixed-point
scale is used (smaller than a raw `float`/`double`). Where the range isn't
well characterized, the field is packed as a full-precision raw `float` to
avoid guessing at a scale that might clip real data.

## Field order

The field order matches the order values are gathered/assigned in
`tickMeasure()`, which is also the order they appear in the existing
`DataLine` debug string:

1. Board ID, elapsed time (ms)
2. Power rails: `bat_v`, `vin`, `charge_i`, `v_5V`
3. Subsystem currents: `pump_i`, `opc_i`, `tsen_i`, `tdlas_i`, `heater_i`
4. Pump status: `bemf_v`, `pwm`
5. GPS: lat, lon, alt, sats, date, time, fix age
6. Board/pump/battery temperatures: `pcb_t`, `pump_t`, `bat_t`
7. OPC: `ROPC_time`, 8 particle-bin counts (`d300`...`d5000`), `alarm`
8. TSEN: `tsen_airt`, `tsen_ptemp`, `tsen_pres` (raw, uncalibrated)
9. TDLAS: `mr_avg`, `bkg`, `peak`, `ratio`, `batt`, `therm_1`, `therm_2`,
   `indx`, `spec_1`...`spec_4`
10. RS41: `frame_count`, `air_temp`, `humidity`, `hsensor_temp`, `pres`,
    `internal_temp`, `module_status`, `module_error`, `pcb_supply_V`,
    `lsm303_temp`, `pcb_heater_on`, magnetic headings (XY/XZ/YZ), accelerations
    (X/Y/Z)

The raw ASCII response string from the TSEN probe (`tsenData`, the
`"#AAA PPPPPP TTTTTT\r"` line itself) is *not* included — it isn't part of the
numeric `DataLine` record either, and a raw ASCII blob doesn't fit a
fixed-width bit-packed field. The three values *parsed* from that string
(`tsen_airt`, `tsen_ptemp`, `tsen_pres`) are included, as raw/uncalibrated
integers — see the TSEN section below.

## Research inputs

Two existing references were consulted to choose realistic ranges/resolutions
before picking bit widths:

### `RS41SensorData_t` (`RS41.h`)

Gives the actual field types coming out of the RS41 driver:
- `double air_temp_degC`, `humdity_percent`, `hsensor_temp_degC`,
  `pres_mb`, `internal_temp_degC`, `lsm303_temp_degC`
- `unsigned int frame_count`, `module_status`, `module_error`,
  `pcb_heater_on`
- `double pcb_supply_V`
- `int32_t mag_hdgXY_deg`, `mag_hdgXZ_deg`, `mag_hdgYZ_deg` — documented range
  0–360°
- `int32_t accelX_mG`, `accelY_mG`, `accelZ_mG`

### `ECUReport_t` (sibling project, `ECUcomm/src/ECUReport.h`)

A bit-packed report with similar goals, used as a precedent for scale
factors:
- Voltages: x100
- Board/CPU temperature: `(T + 100) x10` in 11 bits (-100.0 to 104.8 °C)
- RS41 air temp: `(T + 100) x100` in 14 bits (-100.00 to 63.83 °C)
- RS41 humidity: x10 in 10 bits (0–102.3 %)
- RS41 heater-sensor temp: `(T + 100)` in 8 bits (1 °C res)
- RS41 pressure: x100 in 17 bits (0–1310.71 hPa)
- GPS lat/lon: x1e6 as int32
- GPS fix age: 8 bits, clamped to 0–255 s
- TSEN: `add_tsen()` packs `tsen_airt` (12 bits, 0–0xFFF), `tsen_ptemp` (24
  bits, 0–0xFFFFFF), and `tsen_pres` (24 bits, 0–0xFFFFFF) as raw,
  uncalibrated A/D counts — no physical-unit conversion is applied or
  documented

## Scaling decisions, by group

### Identification / time

| Field | Type | Encoding | Bits |
|---|---|---|---|
| `board_id` | uint16 | raw | 16 |
| `elapsed_ms` | uint32 | raw (`millis()`) | 32 |

### Power rails and currents

`RPUPacket` already used x10 V (0–25.5 V, 8 bits) and mA (0–4095, 12 bits).
For `RPUReport` the voltage scale was widened slightly to **x100 V in 12
bits** (0–40.95 V, 0.01 V resolution) — more headroom and resolution than
`RPUPacket`'s x10/8-bit scheme, justified since there's no size pressure.
This scale is reused for every voltage in the report: `bat_v`, `vin`, `v_5V`,
pump `bemf_v`, and the RS41 `pcb_supply_V`.

`charge_i` is the battery charge current and can exceed the ~4 A range of the
other subsystem currents, so it gets its own **mA, 13 bits** (0–8191 mA)
field rather than reusing the 12-bit current scale.

The remaining subsystem currents (`pump_i`, `opc_i`, `tsen_i`, `tdlas_i`,
`heater_i`) reuse `RPUPacket`'s existing **mA, 12-bit** (0–4095 mA) scale —
these are well-characterized low-power loads.

| Field | Type | Encoding | Bits |
|---|---|---|---|
| `bat_v`, `vin`, `v_5V`, `bemf_v` | float V | x100, 0–40.95 V | 12 each |
| `charge_i` | float A | mA, 0–8191 mA | 13 |
| `pump_i`, `opc_i`, `tsen_i`, `tdlas_i`, `heater_i` | float mA | raw mA, 0–4095 | 12 each |
| `pump.pwm` | uint8 | raw, 0–255 | 8 |

### GPS

These fields are already bit-packed in `RPUPacket` with widths chosen to
match `ECUComm`'s conventions, so `RPUReport` reuses them unchanged:

| Field | Type | Encoding | Bits |
|---|---|---|---|
| `lat` | double | `(lat + 90) x10000` | 21 |
| `lon` | double | `(lon + 180) x10000` | 22 |
| `alt` | float m | raw, 0–65535 m | 16 |
| `sats` | uint32 | raw, 0–31 | 5 |
| `gps_date` | uint32 | DDMMYY | 19 |
| `gps_time` | uint32 | HHMMSSCC | 25 |
| `gps_age` | uint32 | seconds, clamped 0–255 | 8 |

One new GPS-related field not present in `RPUPacket`: GPS fix age
(`location.age()/1000`, seconds). Initially given a full **uint16, 16 bits**
(0–65535 s) so a long-stale fix wouldn't get clamped silently, but later
revised down to match `ECUReport_t`'s **8 bits, clamped to 0–255 s** as part
of trimming `RPUReport`'s overall size — a fix more than 4 minutes stale is
already "very stale" for any practical purpose, so the extra range wasn't
worth 8 bits.

### Board / pump / battery temperatures

`RPUPacket` uses `(T + 100) x2` in 9 bits (-100 to +155.5 °C, 0.5 °C
resolution). For `RPUReport`, this was widened to **`(T + 100) x10` in 12
bits**, giving a range of -100.0 to +309.5 °C at 0.1 °C resolution — finer
resolution than `RPUPacket`, with enough range to comfortably cover
`pcb_t`, `pump_t`, and `bat_t`. This same temperature scale is reused for the
RS41 temperatures below, since the RS41 also reports values that can go well
below 0 °C at altitude.

| Field | Type | Encoding | Bits |
|---|---|---|---|
| `pcb_t`, `pump_t`, `bat_t` | float °C | `(T+100) x10`, -100.0 to 309.5 °C | 12 each |

### OPC (optical particle counter)

These are all raw counters/timestamps with no obvious scaling benefit —
packed as-is, just without byte padding:

| Field | Type | Encoding | Bits |
|---|---|---|---|
| `ROPC_time` | uint32 | raw ms | 32 |
| `d300`...`d5000` (8 fields) | uint16 | raw counts | 16 each |
| `alarm` | uint8 | raw bitfield | 8 |

### TSEN (temperature/pressure sensor)

`readTSEN()` parses the TSEN probe's `"#AAA PPPPPP TTTTTT\r"` response (19
ASCII chars: `#` + 3 hex digits + space + 6 hex digits + space + 6 hex digits
+ `\r`) into three raw, uncalibrated integers — `tsen_airt` (air temperature
A/D count), `tsen_ptemp` (pressure-sensor temperature count), and `tsen_pres`
(pressure count).

No documented calibration/conversion to physical units exists for these
values — `ECUReport_t`'s `add_tsen()` keeps them raw too, so `RPUReport`
follows the same precedent and reuses ECU's bit widths directly (12 bits for
`tsen_airt`'s 0–0xFFF range, 24 bits each for `tsen_ptemp`/`tsen_pres`'s
0–0xFFFFFF range). Setters clamp to these ranges via `constrain()` as a safety
bound.

| Field | Type | Encoding | Bits |
|---|---|---|---|
| `tsen_airt` | uint16 | raw 12-bit A/D count, 0–0xFFF | 12 |
| `tsen_ptemp` | uint32 | raw 24-bit count, 0–0xFFFFFF | 24 |
| `tsen_pres` | uint32 | raw 24-bit count, 0–0xFFFFFF | 24 |

### TDLAS

This was the main open question. The TDLAS fields
(`mr_avg`, `bkg`, `peak`, `ratio`, `batt`, `therm_1`, `therm_2`, `spec_1`-`4`)
are `float`s in `TDLASData` with no documented range or required resolution —
unlike the RS41 fields, there was no reference (`ECUReport_t` doesn't carry
TDLAS data) to derive a safe fixed-point scale from.

Two options were considered:

1. **Fixed-point with a guessed scale/range** — compact, but risks silently
   clipping or losing precision on values whose real-world range isn't known.
2. **Raw IEEE-754 `float32`**, packed via `etl::bit_stream_writer`/`reader`'s
   `write_unchecked<uint32_t>(bits, 32)` (after a `memcpy` from `float` to
   `uint32_t`) — full precision, no risk of clipping, at a cost of 32 bits
   instead of e.g. 16.

Given "compact" was clarified to mean "no padding," not "aggressively
lossy," and given there's no LoRa size constraint to justify the risk,
**option 2 (raw float32) was chosen** for all 11 TDLAS float fields. ETL's
`bit_stream` also separately offers `put<float>()`/`get<float>()`, which pack
the full `sizeof(float)*8` bits via byte-wise `to_bytes`/`from_bytes` — but
that's a different API from the `write_unchecked`/`read_unchecked` calls
`RPUPacket` already uses, so the manual `memcpy`-based helpers
(`writeFloat32`/`readFloat32`) were used instead to keep the whole class on
one consistent API.

`tdlas_indx` (`int8_t`, a small spectrum index) is packed as a raw signed
8-bit value.

| Field | Type | Encoding | Bits |
|---|---|---|---|
| `mr_avg`, `bkg`, `peak`, `ratio`, `batt`, `therm_1`, `therm_2`, `spec_1`-`spec_4` (11 fields) | float | raw IEEE-754 float32 | 32 each |
| `indx` | int8 | raw | 8 |

### RS41

| Field | Type | Encoding | Bits | Notes |
|---|---|---|---|---|
| `frame_count` | uint32 | raw | 32 | |
| `air_temp_degC` | float °C | `(T+100) x10` | 12 | shares the board-temp scale above; `ECUReport_t` used `x100`/14 bits but that's overkill here |
| `humdity_percent` | float % | `x10`, 0–102.3 % | 10 | matches `ECUReport_t` |
| `hsensor_temp_degC` | float °C | `(T+100) x10` | 12 | `ECUReport_t` used 1 °C res / 8 bits; 0.1 °C res chosen here for consistency with the other temps |
| `pres_mb` | float mb | `x100`, 0–1310.71 mb | 17 | matches `ECUReport_t` |
| `internal_temp_degC` | float °C | `(T+100) x10` | 12 | |
| `module_status`, `module_error` | unsigned | raw uint8 | 8 each | |
| `pcb_supply_V` | float V | shares the x100 V / 12-bit voltage scale | 12 | |
| `lsm303_temp_degC` | float °C | `(T+100) x10` | 12 | |
| `pcb_heater_on` | int (bool) | 1-bit flag | 1 | |
| `mag_hdgXY/XZ/YZ_deg` | int32, 0–360° | raw degrees, 1° res | 9 each | 9 bits covers 0–511, comfortably holds 0–360 |
| `accelX/Y/Z_mG` | int32 (mG) | signed int16, 1 mG res | 16 each | ±32767 mG (~±32.8 g) range, far exceeds any expected reading |

## Final bit layout / packet size

Total payload: 4 (version header) + 1128 (fields) = **1132 bits = 142 bytes**
(`RPU_RPT_BYTES`). All constants are named `RPU_RPT_*` in `RPUcomm.h` to keep
them distinct from the `RPU_PKT_*` constants used by `RPUPacket`.

| Constant | Bits | Used for |
|---|---|---|
| `RPU_RPT_VER_BITS` | 4 | packet format version |
| `RPU_RPT_ID_BITS` | 16 | board ID |
| `RPU_RPT_TIME_BITS` | 32 | `elapsed_ms`, `ROPC_time`, RS41 `frame_count` |
| `RPU_RPT_VOLT_BITS` | 12 | all voltages, x100 (0–40.95 V) |
| `RPU_RPT_CHGI_BITS` | 13 | `charge_i`, mA (0–8191 mA) |
| `RPU_RPT_CURR_BITS` | 12 | subsystem currents, mA (0–4095 mA) |
| `RPU_RPT_PWM_BITS` | 8 | pump PWM (0–255) |
| `RPU_RPT_LAT_BITS` | 21 | `(lat+90) x10000` |
| `RPU_RPT_LON_BITS` | 22 | `(lon+180) x10000` |
| `RPU_RPT_ALT_BITS` | 16 | altitude, m |
| `RPU_RPT_SATS_BITS` | 5 | satellite count |
| `RPU_RPT_GPS_DATE_BITS` | 19 | DDMMYY |
| `RPU_RPT_GPS_TIME_BITS` | 25 | HHMMSSCC |
| `RPU_RPT_GPS_AGE_BITS` | 8 | GPS fix age, s, clamped (0–255 s) |
| `RPU_RPT_TEMP_BITS` | 12 | `(T+100) x10` (-100.0 to 309.5 °C) |
| `RPU_RPT_HUM_BITS` | 10 | RS41 humidity, x10 (0–102.3 %) |
| `RPU_RPT_PRES_BITS` | 17 | RS41 pressure, x100 (0–1310.71 mb) |
| `RPU_RPT_OPC_BITS` | 16 | OPC bin counts, raw |
| `RPU_RPT_ALARM_BITS` | 8 | OPC alarm flags |
| `RPU_RPT_TSEN_AIRT_BITS` | 12 | TSEN air temp, raw A/D count (0–0xFFF) |
| `RPU_RPT_TSEN_PTEMP_BITS` | 24 | TSEN pressure-sensor temp, raw count (0–0xFFFFFF) |
| `RPU_RPT_TSEN_PRES_BITS` | 24 | TSEN pressure, raw count (0–0xFFFFFF) |
| `RPU_RPT_F32_BITS` | 32 | raw float32 (TDLAS) |
| `RPU_RPT_INDX_BITS` | 8 | TDLAS spectrum index |
| `RPU_RPT_STATUS_BITS` | 8 | RS41 module status/error |
| `RPU_RPT_HDG_BITS` | 9 | RS41 magnetic heading, deg (0–360) |
| `RPU_RPT_ACCEL_BITS` | 16 | RS41 acceleration, mG (signed) |

## Open items / not yet done

- `RPUReport` is not yet wired into `RPU.cpp`'s `tickMeasure()`, and there is
  no `TX`/`RX` method on `RPUComm` to send/receive it over `DOCK_SERIAL` yet.
- The `-Wformat-truncation` warning GCC emits for `RPUReport::toJSON()` is a
  known false positive: it assumes the worst case for `%f` with a `double`
  (the type floats are promoted to in varargs), which would require an
  unreasonably large buffer to fully silence. The actual TDLAS/RS41 values
  are physically bounded, so this was left as-is.
