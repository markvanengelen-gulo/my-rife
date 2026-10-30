# Frequency Player

A single-page web app that plays tones through your device speaker using the Web Audio API.

## Features
- Preset frequency lists (commonly cited Rife list frequencies, Solfeggio, Schumann, test tones)
- Pick one frequency or step through a whole list, with a set number of seconds each
- Sine / square / triangle / sawtooth waveforms and a volume control
- "My list": add your own frequencies, saved in the browser on that device

## Run it
Open `index.html` in any modern browser (works on phones). No build step or install needed.

To host it, enable **GitHub Pages** on this repo (Settings → Pages → deploy from `main`, root).

## Notes
- Most phone speakers/earbuds reproduce roughly 50 Hz – 18 kHz; tones outside that may be inaudible.
- Keep volume moderate; long exposure to pure tones can strain hearing.
- This is an audio tool. Rife frequencies have no scientific evidence of health effects, and nothing here is medical advice.

## Watch firmware
`firmware/` holds firmware for the LilyGO T-Watch S3 with the same frequency lists (speaker for 40 Hz and up, vibration below; LoRa, WiFi and Bluetooth off). Install it from `flash.html` in Chrome or Edge on a computer. See `firmware/README.md`.
