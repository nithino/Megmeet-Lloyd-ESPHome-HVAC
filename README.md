# Megmeet-Lloyd-ESPHome-HVAC
UART based ESPHome component for Lloyd ACs using Megmeet inverter AC boards.

## Status (2026-11-23)
* **Reading and writing work.** The TX fault was a wiring problem (fixed 2026-11-23). The AC answers heartbeats,
  product query (`Meg_Lloyd_AC_Gen`), working mode and DP queries, and echoes every accepted DP write within ~15-40 ms.
* **Climate entity works**: `climate: - platform: megmeet_tuya` gives power, mode, target temperature, fan and
  room temperature. State follows the AC's own reports, so IR-remote changes show up too.

## Protocol (what is actually on the wire)
The Wi-Fi module header speaks **9600 baud, 8N1, parity NONE**, Tuya-style framing:

```
55 AA | VER | CMD | LEN_HI LEN_LO | DATA ... | CHECKSUM (sum of all previous bytes, & 0xFF)
```

The AC sends `VER=00 CMD=07` (datapoint report) frames on its own about once per second, one 4-byte
integer DP per frame, e.g. `55 AA 00 07 00 08 6D 02 00 04 00 00 00 1A 9B` = DP 109 (0x6D) = 26.
With `handshake: true` (default) the component sends a heartbeat every 15 s and, on boot and after an AC restart,
runs product query -> working mode -> network status (0x03 = 04) -> DP query (0x08). It also answers the AC's
network-status (0x2B) and local-time (0x1C) requests.

> The older `megmeet_uart` component in this repo decodes "55 35 ..." 10-byte frames. Those were captured with
> the UART set to EVEN parity, which was wrong, so that decoding is obsolete. It is kept only for history.

## Datapoints seen (AC running in Cool)
| DP | Example | Probable meaning |
|----|---------|------------------|
| 109 | 26-27 | Room (return air) temperature, deg C - matches a separate room sensor |
| 124 | 10-11 | Indoor coil (evaporator) temperature, deg C |
| 135 | 30-32 | Outdoor ambient temperature, deg C |
| 133 | 36-37 | Probably outdoor coil (condenser) temperature, deg C |
| 134 | 62-63 | Probably compressor discharge temperature, deg C |
| 122 | 242-244 | Mains supply voltage, V |
| 136 | 376-380 | DC bus voltage (after PFC), V |
| 112 | 459-463 | unknown |
| 138 | 111-112 | unknown |
| 131 | 2-3 | unknown (state/step) |
| 123 | 21 | unknown |

| 137 | 0-38 | Compressor frequency, Hz (0 = compressor stopped) |

## Control datapoints (confirmed by write + echo, 2026-11-23)
| DP | Type | Meaning | Values |
|----|------|---------|--------|
| 1 | bool | Power | 0 off, 1 on |
| 2 | value | Target temperature | 16-31 deg C, step 1 (15 and 32 rejected) |
| 4 | enum | Mode | 0 auto, 1 cool, 3 dry (forces fan 0), 4 fan only; 2 (heat) rejected - cooling-only unit |
| 5 | enum | Fan | 0 auto, 1-4 accepted (1 low, 2 medium, 3 high, 4 max assumed, not verified physically); 5 rejected |

These are only sent in answer to a DP query or as the echo of a write, never unsolicited. Swing / turbo / eco DPs
are not mapped yet.

## TX problem (solved)
Phase 1 (2026-11-08) found the ESP TX line floating with no load: the wire did not reach the AC's RX input.
After the wiring was fixed (2026-11-23) TX worked at normal (non-inverted) polarity.

## Usage
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/nithino/Megmeet-Lloyd-ESPHome-HVAC
      ref: main
    components: [megmeet_tuya]

uart:
  id: uart_bus
  tx_pin: GPIO20
  rx_pin: GPIO21
  baud_rate: 9600
  parity: NONE

megmeet_tuya:
  id: lloyd
  uart_id: uart_bus
  # handshake: true          # default: heartbeats + init sequence
  # heartbeat_interval: 15s
  # link_timeout: 30s

climate:
  - platform: megmeet_tuya
    name: "Master AC"
    power_datapoint: 1
    target_temperature_datapoint: 2
    current_temperature_datapoint: 109
    mode_datapoint: 4
    modes:
      auto: 0
      cool: 1
      dry: 3
      fan_only: 4
    fan_mode_datapoint: 5
    fan_modes:
      auto: 0
      low: 1
      medium: 2
      high: 3
    custom_fan_modes:
      Max: 4
    visual:
      min_temperature: 16
      max_temperature: 31
      temperature_step: 1

binary_sensor:
  - platform: megmeet_tuya
    name: "Master AC UART Link"

sensor:
  - platform: megmeet_tuya
    datapoint: 109
    name: "Master AC Room Temperature"
    unit_of_measurement: "°C"
    device_class: temperature
```

TX helpers for lambdas: `id(lloyd).send_command(cmd, {payload}, version)`, `set_dp_bool(dp, v)`,
`set_dp_value(dp, v)`, `set_dp_enum(dp, v)`, `write_dp(dp, v, fallback_type)`, `query_dps()`, `dump_dps()`,
`send_raw({bytes})`, `set_tx_inverted(gpio, bool)`.
See `megmeet_tuya.yaml` for the full device example.
