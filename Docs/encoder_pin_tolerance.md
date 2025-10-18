# Encoder Input Voltage Tolerance Verification

This note records the 5 V compatibility of the STM32L475 encoder inputs used by the CNC controller. The information comes from the STM32L47x/48x datasheet (DS11585, Rev. 7) tables "Alternate function" and the accompanying footnotes that define the I/O type abbreviations (`FT`, `FT_a`). In those tables `FT` denotes a 5 V tolerant I/O when VDD is present, and `FT_a` denotes the same tolerance except when the pad is configured in analog mode, oscillator mode, or when any internal pull-up/pull-down is enabled.

| Timer/Axis | MCU pin | Datasheet I/O type | 5 V tolerant conditions |
|------------|---------|--------------------|-------------------------|
| TIM2 / X   | PA15    | `FT`               | 5 V tolerant in all digital modes as long as VDD is applied. |
| TIM2 / X   | PB3     | `FT`               | 5 V tolerant in all digital modes as long as VDD is applied. |
| TIM3 / Z   | PE3     | `FT`               | 5 V tolerant in all digital modes as long as VDD is applied. |
| TIM3 / Z   | PE4     | `FT`               | 5 V tolerant in all digital modes as long as VDD is applied. |
| TIM5 / Y   | PA0     | `FT_a`             | 5 V tolerant when configured as digital/alternate function with no internal pull-up or pull-down and not used as analog input. |
| TIM5 / Y   | PA1     | `FT_a`             | 5 V tolerant when configured as digital/alternate function with no internal pull-up or pull-down and not used as analog input. |

Because the firmware drives each encoder channel as an alternate-function input, it must leave the internal pull resistors disabled to respect the `FT_a` restriction on PA0/PA1. The CubeMX-generated initialization already uses `GPIO_NOPULL`; no further electrical changes are required, but this file documents the datasheet justification for keeping those pins in high-impedance mode.
