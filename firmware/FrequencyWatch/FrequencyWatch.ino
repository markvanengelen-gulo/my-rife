/*
  Frequency Watch — firmware for the LilyGO T-Watch S3 (tested build target: K222, 915 MHz)

  - Plays tones from the same frequency lists as the web app (index.html)
  - 40 Hz and up  -> tone through the speaker (MAX98357A I2S amplifier)
  - Below 40 Hz   -> pulses of the vibration motor (DRV2605) at that rate,
                     because no watch speaker can reproduce those frequencies
  - Radio silent: the LoRa radio's power rail is switched off, and WiFi and
    Bluetooth are never started.

  Controls
  - List screen: < > change list, tap a frequency to play it, UP/DN scroll,
    TIME cycles the length of each tone, ALL plays the whole list.
  - Player screen: STOP, - / + volume, NEXT (when playing a list).
  - Screen turns off after 20 s; tap or short-press the side button to wake.
  - Hold the side button ~6 s to power off; press ~2 s to power on.

  This is an audio/vibration tool. It is not a medical device and has no
  proven health effect.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include <LovyanGFX.hpp>
#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>
#include <Adafruit_DRV2605.h>
#include "frequencies.h"

// ---------- Pins (LilyGO T-Watch S3 hardware reference) ----------
#define PIN_I2C_SDA    10
#define PIN_I2C_SCL    11
#define PIN_TOUCH_SDA  39
#define PIN_TOUCH_SCL  40
#define PIN_DISP_SCK   18
#define PIN_DISP_MOSI  13
#define PIN_DISP_CS    12
#define PIN_DISP_DC    38
#define PIN_DISP_BL    45
#define PIN_I2S_BCLK   48
#define PIN_I2S_WCLK   15
#define PIN_I2S_DOUT   46
#define TOUCH_ADDR     0x38

// If the screen's red and blue look swapped, set this to 1.
#define DISPLAY_RGB_ORDER 0
// If taps land in the wrong place, adjust these.
#define TOUCH_SWAP_XY  0
#define TOUCH_MIRROR_X 0
#define TOUCH_MIRROR_Y 0

#define SAMPLE_RATE     96000   // 96 kHz: ~4.8 samples per cycle even at 20 kHz
#define VIBE_BELOW_HZ   40.0f
#define SCREEN_TIMEOUT  20000

// ---------- Display ----------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI _bus;
public:
  LGFX() {
    auto b = _bus.config();
    b.spi_host = SPI2_HOST;
    b.spi_mode = 0;
    b.freq_write = 40000000;
    b.pin_sclk = PIN_DISP_SCK;
    b.pin_mosi = PIN_DISP_MOSI;
    b.pin_miso = -1;
    b.pin_dc = PIN_DISP_DC;
    b.dma_channel = SPI_DMA_CH_AUTO;
    _bus.config(b);
    _panel.setBus(&_bus);

    auto p = _panel.config();
    p.pin_cs = PIN_DISP_CS;
    p.pin_rst = -1;
    p.pin_busy = -1;
    p.panel_width = 240;
    p.panel_height = 240;
    p.memory_width = 240;
    p.memory_height = 320;
    p.offset_x = 0;
    p.offset_y = 0;
    p.invert = true;
    p.rgb_order = DISPLAY_RGB_ORDER;
    p.readable = false;
    p.bus_shared = false;
    _panel.config(p);
    setPanel(&_panel);
  }
};

LGFX lcd;
XPowersPMU pmu;
Adafruit_DRV2605 drv;
Preferences prefs;

// Colours
#define C_BG     0x0841
#define C_CARD   0x1904
#define C_LINE   0x2986
#define C_TEXT   0xE73C
#define C_MUTED  0x8CD4
#define C_ACCENT 0x4D95
#define C_STOP   0xDB29

// ---------- Settings ----------
static const uint16_t DURATIONS[] = {60, 120, 180, 300, 600};
static uint8_t durIdx = 2;     // 180 s
static uint8_t volume = 5;     // 1..10

// ---------- Audio (runs in its own task) ----------
static volatile float g_freq = 440.0f;
static volatile bool  g_tone = false;
static volatile float g_amp  = 0.0f;

void audioTask(void *) {
  const int N = 256;
  static int16_t buf[N * 2];
  // Exact 32-bit phase accumulator: frequency resolution of SAMPLE_RATE / 2^32
  // (~0.00002 Hz) and no drift, however long a tone plays.
  uint32_t phase = 0;
  float cur = 0;
  bool running = true;
  for (;;) {
    float target = g_tone ? g_amp : 0.0f;
    if (target == 0.0f && cur < 0.0005f) {
      if (running) { i2s_zero_dma_buffer(I2S_NUM_0); i2s_stop(I2S_NUM_0); running = false; }
      cur = 0;
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (!running) { i2s_start(I2S_NUM_0); running = true; }
    const uint32_t step = (uint32_t)((double)g_freq / SAMPLE_RATE * 4294967296.0 + 0.5);
    const float toRad = 2.0f * PI / 4294967296.0f;
    for (int i = 0; i < N; i++) {
      cur += (target - cur) * 0.0015f;             // smooth fade, no clicks
      int16_t v = (int16_t)(sinf((float)phase * toRad) * cur * 32000.0f);
      buf[2 * i] = buf[2 * i + 1] = v;
      phase += step;                               // wraps exactly at 2^32 = one cycle
    }
    size_t written;
    i2s_write(I2S_NUM_0, buf, sizeof(buf), &written, portMAX_DELAY);
  }
}

void initAudio() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.tx_desc_auto_clear = true;
  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_WCLK;
  pins.data_out_num = PIN_I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;
  i2s_set_pin(I2S_NUM_0, &pins);

  xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, 5, nullptr, 0);
}

float volumeToAmp() { return (volume * volume) / 100.0f * 0.9f; }

// ---------- Vibration ----------
static bool vibeActive = false, vibeOn = false;
static uint32_t vibeToggleUs = 0, vibeHalfUs = 0;

void vibeWrite(bool on) {
  vibeOn = on;
  drv.setRealtimeValue(on ? (uint8_t)(0x30 + volume * 0x4F / 10) : 0);
}

// ---------- Backlight ----------
static bool screenOn = true;
static uint32_t lastActivity = 0;

void setBacklight(bool on) {
  screenOn = on;
  ledcWrite(0, on ? 200 : 0);
}

// ---------- Touch (FT6336U on its own I2C bus) ----------
bool readTouch(int16_t &x, int16_t &y) {
  Wire1.beginTransmission(TOUCH_ADDR);
  Wire1.write(0x02);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(TOUCH_ADDR, 5) != 5) return false;
  uint8_t n = Wire1.read() & 0x0F;
  uint8_t xh = Wire1.read(), xl = Wire1.read(), yh = Wire1.read(), yl = Wire1.read();
  if (n == 0 || n > 2) return false;
  x = ((xh & 0x0F) << 8) | xl;
  y = ((yh & 0x0F) << 8) | yl;
#if TOUCH_SWAP_XY
  int16_t t = x; x = y; y = t;
#endif
#if TOUCH_MIRROR_X
  x = 239 - x;
#endif
#if TOUCH_MIRROR_Y
  y = 239 - y;
#endif
  return true;
}

// ---------- App state ----------
enum Screen { LIST, PLAYER };
static Screen screen = LIST;
static uint8_t listIdx = 0, scroll = 0;
static int curItem = -1;
static bool playAll = false;
static uint32_t startMs = 0, durMs = 0;
static int lastSecShown = -1;

const FreqList &L() { return LISTS[listIdx]; }

String hzText(float hz) {
  if (hz == (int)hz) return String((int)hz);
  return String(hz, 2);
}

// ---------- Drawing ----------
void button(int x, int y, int w, int h, const char *t, uint16_t bg, uint16_t fg = C_TEXT) {
  lcd.fillRoundRect(x + 2, y + 2, w - 4, h - 4, 8, bg);
  lcd.setTextColor(fg, bg);
  lcd.setTextDatum(middle_center);
  lcd.setFont(&fonts::Font2);
  lcd.drawString(t, x + w / 2, y + h / 2);
}

void drawList() {
  lcd.fillScreen(C_BG);
  // header
  button(0, 0, 40, 36, "<", C_CARD);
  button(200, 0, 40, 36, ">", C_CARD);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.setTextDatum(middle_center);
  lcd.setFont(&fonts::Font2);
  String name = L().name;
  if (name.length() > 19) name = name.substring(0, 18) + ".";
  lcd.drawString(name, 120, 18);

  // rows
  for (int r = 0; r < 4; r++) {
    int i = scroll + r, y = 38 + r * 40;
    if (i >= L().count) break;
    lcd.fillRoundRect(2, y, 236, 38, 6, C_CARD);
    const Freq &f = L().items[i];
    lcd.setTextDatum(top_left);
    lcd.setFont(&fonts::Font2);
    lcd.setTextColor(C_ACCENT, C_CARD);
    String hz = hzText(f.hz) + " Hz";
    if (f.hz < VIBE_BELOW_HZ) hz += "  (vibrate)";
    lcd.drawString(hz, 10, y + 3);
    lcd.setFont(&fonts::Font0);
    lcd.setTextColor(C_MUTED, C_CARD);
    String lb = f.label;
    if (lb.length() > 37) lb = lb.substring(0, 36) + ".";
    lcd.drawString(lb, 10, y + 24);
  }

  // footer
  char t[8];
  snprintf(t, sizeof(t), "%dm", DURATIONS[durIdx] >= 60 ? DURATIONS[durIdx] / 60 : 1);
  button(0, 200, 60, 40, "UP", C_CARD);
  button(60, 200, 60, 40, "DN", C_CARD);
  button(120, 200, 60, 40, t, C_CARD);
  button(180, 200, 60, 40, "ALL", C_ACCENT, 0x0000);
}

void drawPlayerStatic() {
  const Freq &f = L().items[curItem];
  lcd.fillScreen(C_BG);
  lcd.setTextColor(C_MUTED, C_BG);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_left);
  char top[32];
  if (playAll) snprintf(top, sizeof(top), "%d of %d", curItem + 1, L().count);
  else snprintf(top, sizeof(top), "Single");
  lcd.drawString(top, 6, 4);
  lcd.setTextDatum(top_right);
  char vb[24];
  snprintf(vb, sizeof(vb), "Vol %d  %d%%", volume, pmu.getBatteryPercent());
  lcd.drawString(vb, 234, 4);

  // big number
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.setFont(&fonts::Font7);
  String hz = hzText(f.hz);
  lcd.setTextSize(hz.length() > 4 ? 0.8f : 1.0f);
  lcd.drawString(hz, 120, 62);
  lcd.setTextSize(1);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(C_MUTED, C_BG);
  const char *how = f.hz < VIBE_BELOW_HZ ? "Hz - vibration" : (f.hz > 18000 ? "Hz - above most speakers" : "Hz - speaker");
  lcd.drawString(how, 120, 100);

  // label (up to two lines)
  lcd.setTextColor(C_TEXT, C_BG);
  String lb = f.label;
  String l1 = lb, l2 = "";
  if (lb.length() > 26) {
    int cut = lb.lastIndexOf(',', 26);
    if (cut < 0) cut = 26;
    l1 = lb.substring(0, cut + 1);
    l2 = lb.substring(cut + 1);
    l2.trim();
    if (l2.length() > 28) l2 = l2.substring(0, 27) + ".";
  }
  lcd.drawString(l1, 120, 124);
  if (l2.length()) lcd.drawString(l2, 120, 142);

  // buttons
  button(0, 196, 60, 44, "STOP", C_STOP);
  button(60, 196, 60, 44, "-", C_CARD);
  button(120, 196, 60, 44, "+", C_CARD);
  if (playAll) button(180, 196, 60, 44, "NEXT", C_CARD);
  lastSecShown = -1;
}

void drawPlayerProgress() {
  uint32_t el = millis() - startMs;
  int left = (int)((durMs > el ? durMs - el : 0) / 1000);
  if (left == lastSecShown) return;
  lastSecShown = left;
  int w = durMs ? (int)(220.0f * el / durMs) : 0;
  if (w > 220) w = 220;
  lcd.fillRoundRect(10, 162, 220, 8, 4, C_LINE);
  lcd.fillRoundRect(10, 162, w, 8, 4, C_ACCENT);
  char t[12];
  snprintf(t, sizeof(t), "%d:%02d", left / 60, left % 60);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(C_MUTED, C_BG);
  lcd.fillRect(80, 174, 80, 18, C_BG);
  lcd.drawString(t, 120, 175);
}

// ---------- Playback ----------
void stopOutput() {
  g_tone = false;
  vibeActive = false;
  vibeWrite(false);
}

void startItem(int i) {
  stopOutput();
  curItem = i;
  const Freq &f = L().items[i];
  startMs = millis();
  durMs = DURATIONS[durIdx] * 1000UL;
  if (f.hz < VIBE_BELOW_HZ) {
    vibeActive = true;
    vibeHalfUs = (uint32_t)(500000.0f / f.hz);
    vibeToggleUs = micros();
    vibeWrite(true);
  } else {
    g_freq = f.hz;
    g_amp = volumeToAmp();
    g_tone = true;
  }
  screen = PLAYER;
  drawPlayerStatic();
  drawPlayerProgress();
}

void stopPlayback() {
  stopOutput();
  curItem = -1;
  screen = LIST;
  drawList();
}

void saveSettings() {
  prefs.putUChar("vol", volume);
  prefs.putUChar("dur", durIdx);
}

// ---------- Input ----------
void onTap(int16_t x, int16_t y) {
  if (screen == LIST) {
    if (y < 36) {
      if (x < 60) listIdx = (listIdx + LIST_COUNT - 1) % LIST_COUNT;
      else if (x > 180) listIdx = (listIdx + 1) % LIST_COUNT;
      else return;
      scroll = 0;
      drawList();
    } else if (y < 198) {
      int i = scroll + (y - 38) / 40;
      if (i < L().count) { playAll = false; startItem(i); }
    } else {
      if (x < 60) { if (scroll > 0) { scroll = scroll >= 4 ? scroll - 4 : 0; drawList(); } }
      else if (x < 120) { if (scroll + 4 < L().count) { scroll += 4; drawList(); } }
      else if (x < 180) { durIdx = (durIdx + 1) % (sizeof(DURATIONS) / sizeof(DURATIONS[0])); saveSettings(); drawList(); }
      else { playAll = true; startItem(0); }
    }
  } else {
    if (y < 196) return;
    if (x < 60) stopPlayback();
    else if (x < 180) {
      if (x < 120 && volume > 1) volume--;
      else if (x >= 120 && volume < 10) volume++;
      saveSettings();
      if (g_tone) g_amp = volumeToAmp();
      if (vibeActive && vibeOn) vibeWrite(true);
      drawPlayerStatic();
      drawPlayerProgress();
    } else if (playAll) {
      if (curItem + 1 < L().count) startItem(curItem + 1);
      else stopPlayback();
    }
  }
}

// ---------- Power ----------
void initPower() {
  if (!pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, PIN_I2C_SDA, PIN_I2C_SCL)) {
    Serial.println("PMU not found");
    return;
  }
  pmu.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
  pmu.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_900MA);
  pmu.setSysPowerDownVoltage(2600);

  pmu.setALDO2Voltage(3300); pmu.enableALDO2();   // display backlight
  pmu.setALDO3Voltage(3300); pmu.enableALDO3();   // display + touch
  pmu.setBLDO2Voltage(3300); pmu.enableBLDO2();   // DRV2605 enable
  pmu.disableALDO4();                              // LoRa radio: OFF

  // unused rails
  pmu.disableDC2(); pmu.disableDC5(); pmu.disableALDO1();
  pmu.disableCPUSLDO(); pmu.disableDLDO1();

  // battery: LilyGO says keep charge current under 130 mA
  pmu.disableTSPinMeasure();
  pmu.enableBattDetection();
  pmu.enableBattVoltageMeasure();
  pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_100MA);
  pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
  pmu.setPowerKeyPressOffTime(XPOWERS_POWEROFF_6S);

  pmu.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  pmu.clearIrqStatus();
  pmu.enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ);
}

void setup() {
  Serial.begin(115200);
  // WiFi and Bluetooth are never initialised in this firmware.

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  initPower();
  delay(50);

  Wire1.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(PIN_DISP_BL, 0);
  lcd.init();
  lcd.setRotation(0);
  setBacklight(true);

  if (drv.begin(&Wire)) {
    drv.selectLibrary(1);
    drv.useERM();
    drv.setMode(DRV2605_MODE_REALTIME);
    drv.setRealtimeValue(0);
  } else {
    Serial.println("DRV2605 not found");
  }

  prefs.begin("freqwatch", false);
  volume = constrain(prefs.getUChar("vol", 5), 1, 10);
  durIdx = prefs.getUChar("dur", 2) % (sizeof(DURATIONS) / sizeof(DURATIONS[0]));

  initAudio();
  drawList();
  lastActivity = millis();
}

void loop() {
  static bool wasTouched = false;
  static uint32_t lastPmuPoll = 0;
  uint32_t now = millis();

  // touch: act on the moment of pressing
  int16_t x, y;
  bool touched = readTouch(x, y);
  if (touched && !wasTouched) {
    lastActivity = now;
    if (!screenOn) setBacklight(true);   // first tap only wakes the screen
    else onTap(x, y);
  }
  wasTouched = touched;

  // side button: short press toggles the screen
  if (now - lastPmuPoll > 100) {
    lastPmuPoll = now;
    pmu.getIrqStatus();
    if (pmu.isPekeyShortPressIrq()) {
      setBacklight(!screenOn);
      lastActivity = now;
    }
    pmu.clearIrqStatus();
  }

  if (screenOn && now - lastActivity > SCREEN_TIMEOUT) setBacklight(false);

  // vibration pulses for low frequencies
  if (vibeActive) {
    uint32_t us = micros();
    if (us - vibeToggleUs >= vibeHalfUs) {
      vibeToggleUs += vibeHalfUs;
      vibeWrite(!vibeOn);
    }
  }

  // timing / auto-advance
  if (screen == PLAYER && curItem >= 0) {
    if (now - startMs >= durMs) {
      if (playAll && curItem + 1 < L().count) startItem(curItem + 1);
      else stopPlayback();
    } else if (screenOn) {
      drawPlayerProgress();
    }
  }

  delay(vibeActive ? 1 : 15);
}
