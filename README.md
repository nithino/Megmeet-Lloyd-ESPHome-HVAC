# Megmeet-Lloyd-ESPHome-HVAC
UART based ESPHome component for Lloyd ACs using Megmeet inverter AC boards.

## Status (2026-11-08)
* **Reading works** with the new `megmeet_tuya` component.
* **Writing does not work yet**: the AC has never reacted to anything sent to it (see "TX problem" below).

## Protocol (what is actually on the wire)
The Wi-Fi module header speaks **9600 baud, 8N1, parity NONE**, Tuya-style framing:

```
55 AA | VER | CMD | LEN_HI LEN_LO | DATA ... | CHECKSUM (sum of all previous bytes, & 0xFF)
```

The AC sends `VER=00 CMD=07` (datapoint report) frames on its own about once per second, one 4-byte
integer DP per frame, e.g. `55 AA 00 07 00 08 6D 02 00 04 00 00 00 1A 9B` = DP 109 (0x6D) = 26.
It never sends heartbeats or queries, and it has never answered ours.

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

No standard Tuya AC datapoints (1 power, 2 set temp, 4 mode, 5 fan) are ever sent unsolicited.

## TX problem
The ESP's TX pin was verified to toggle correctly (pad read-back), but with TX output disabled the wire shows
**no external pull-up and floats**, i.e. it looks like it is not connected to the AC's RX input. Heartbeats,
product/DP queries and DP writes in both version 0x00 and 0x03, plus an inverted TX signal, all got no reaction.
Check the TX wiring / header pinout before going further.

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
  # heartbeat_interval: 0s   # default: no heartbeats
  # link_timeout: 30s

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

TX helpers for lambdas (for testing once TX is fixed): `id(lloyd).send_command(cmd, {payload}, version)`,
`set_dp_bool(dp, v)`, `set_dp_value(dp, v)`, `set_dp_enum(dp, v)`, `send_raw({bytes})`, `set_tx_inverted(gpio, bool)`.
See `megmeet_tuya.yaml` for the full device example.
