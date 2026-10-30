# Frequency Watch firmware (LilyGO T-Watch S3)

Firmware that turns a LilyGO T-Watch S3 (e.g. K222, 915 MHz) into a standalone frequency player, using the same lists as the web app.

## Install (easiest)
Open `https://markvanengelen-gulo.github.io/my-rife/flash.html` in **Chrome or Edge on a computer**, plug in the watch with a data USB cable, and click **Install on my watch**.

## What it does
- **40 Hz and up:** sine tone through the speaker (MAX98357A amplifier). Realistically audible from a few hundred Hz to roughly 15 kHz on a speaker this small.
- **Below 40 Hz:** the vibration motor (DRV2605) pulses on/off at that rate, since no watch speaker can reproduce those frequencies.
- **Radio silent:** the LoRa radio's power rail (AXP2101 ALDO4) is switched off; WiFi and Bluetooth are never started (their start-up code isn't linked into the firmware at all).
- Battery charging is limited to 100 mA, per LilyGO's guidance to stay under 130 mA.
- Volume and tone length are remembered between restarts.

## Controls
| Screen | Control | Action |
|---|---|---|
| List | `<` `>` | Change list |
| List | Tap a frequency | Play it |
| List | `UP` / `DN` | Scroll |
| List | `3m` (etc.) | Cycle tone length: 1, 2, 3, 5, 10 min |
| List | `ALL` | Play the whole list in order |
| Player | `STOP` / `-` / `+` / `NEXT` | Stop, volume, skip |
| Any | Side button short press | Screen on/off (playback continues) |
| Any | Side button hold 6 s / 2 s | Power off / on |

The screen turns off after 20 s; tap to wake.

## Building it yourself
Arduino IDE with the **esp32 by Espressif 2.0.x** core. Board: *ESP32S3 Dev Module*, USB CDC On Boot: Enabled, Flash Size: 16MB, PSRAM: OPI.
Libraries: LovyanGFX, XPowersLib, Adafruit DRV2605 (+ Adafruit BusIO).

To rebuild the one-file image used by the install page:
```
esptool.py --chip esp32s3 merge_bin -o build/FrequencyWatch-merged.bin --flash_mode dio --flash_freq 80m --flash_size 16MB \
  0x0 FrequencyWatch.ino.bootloader.bin 0x8000 FrequencyWatch.ino.partitions.bin 0xe000 boot_app0.bin 0x10000 FrequencyWatch.ino.bin
```

## If something looks off
Pins come from LilyGO's official T-Watch S3 hardware reference. The firmware compiles cleanly but hasn't been run on a watch yet. Settings at the top of `FrequencyWatch.ino` fix the likely first-run issues:
- Red and blue swapped → `DISPLAY_RGB_ORDER 1`
- Taps landing in the wrong spot → `TOUCH_SWAP_XY` / `TOUCH_MIRROR_X` / `TOUCH_MIRROR_Y`

To go back to LilyGO's factory firmware, see the `firmware` folder of [Xinyuan-LilyGO/LilyGoLib](https://github.com/Xinyuan-LilyGO/LilyGoLib).

This is an audio/vibration tool, not a medical device, and has no proven health effect.
