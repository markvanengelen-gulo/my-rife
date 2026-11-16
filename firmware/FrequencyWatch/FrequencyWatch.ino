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
  - Buttons fire after being held for 0.5 s (a bar fills to show progress).
  - First start: touch alignment (hold three + targets). Hold the side button
    ~2 s on the list screen to redo it.
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
bool touchPresent() {
  Wire1.beginTransmission(TOUCH_ADDR);
  return Wire1.endTransmission() == 0;
}

bool readTouch(int16_t &x, int16_t &y) {
  Wire1.beginTransmission(TOUCH_ADDR);
  Wire1.write(0x02);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(TOUCH_ADDR, 5) != 5) return false;
  uint8_t n = Wire1.read() & 0x0F;
  uint8_t xh = Wire1.read(), xl = Wire1.read(), yh = Wire1.read(), yl = Wire1.read();
  if (n == 0 || n > 2) return false;
  uint8_t ev = xh >> 6;              // 0 = press down, 1 = lift up, 2 = contact, 3 = none
  if (ev == 1 || ev == 3) return false;
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
enum Screen { LIST, PLAYER, TOUCHTEST };
static Screen screen = LIST;
static uint8_t listIdx = 0, scroll = 0;
static int curItem = -1;
static bool playAll = false;
static uint32_t startMs = 0, durMs = 0;
static int lastSecShown = -1;
static uint32_t tickUntil = 0;

const FreqList &L() { return LISTS[listIdx]; }
const uint8_t ROWS = 4;

String hzText(float hz) {
  if (hz == (int)hz) return String((int)hz);
  return String(hz, 2);
}

// ---------- Buttons ----------
// Every tappable thing on screen is a Btn. The same table is used to draw it
// and to decide what a tap hit, so what you see is exactly what you can press.
enum BtnId : uint8_t {
  B_NONE, B_PREV, B_NEXTLIST, B_TITLE, B_ROW0, B_ROW1, B_ROW2, B_ROW3,
  B_UP, B_DOWN, B_TIME, B_ALL, B_STOP, B_VOLDN, B_VOLUP, B_SKIP, B_EXIT
};
struct Btn { int16_t x, y, w, h; BtnId id; bool enabled; };
static Btn btns[12];
static uint8_t btnCount = 0;

void addBtn(int x, int y, int w, int h, BtnId id, bool en = true) {
  if (btnCount < 12) btns[btnCount++] = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h, id, en};
}

// Hit test with a few pixels of slack, and nearest-button fallback so taps
// on gaps or at the very edge of the glass still land on something sensible.
const Btn *hitTest(int16_t x, int16_t y) {
  const Btn *best = nullptr;
  int bestD = 1 << 30;
  for (uint8_t i = 0; i < btnCount; i++) {
    const Btn &b = btns[i];
    int dx = x < b.x ? b.x - x : (x >= b.x + b.w ? x - (b.x + b.w - 1) : 0);
    int dy = y < b.y ? b.y - y : (y >= b.y + b.h ? y - (b.y + b.h - 1) : 0);
    int d = dx * dx + dy * dy;
    if (d < bestD) { bestD = d; best = &b; }
  }
  return (best && bestD <= 12 * 12) ? best : nullptr;
}

void drawBtn(int x, int y, int w, int h, const char *t, uint16_t bg, uint16_t fg, bool en = true) {
  if (!en) { bg = C_BG; fg = C_LINE; }
  lcd.fillRoundRect(x + 2, y + 2, w - 4, h - 4, 10, bg);
  if (!en) lcd.drawRoundRect(x + 2, y + 2, w - 4, h - 4, 10, C_LINE);
  lcd.setTextColor(fg, bg);
  lcd.setTextDatum(middle_center);
  lcd.setFont(&fonts::Font2);
  lcd.drawString(t, x + w / 2, y + h / 2);
}

// Outline a button while the finger is on it.
void highlight(const Btn &b, bool on) {
  uint16_t c = on ? C_TEXT : C_BG;
  lcd.drawRoundRect(b.x + 2, b.y + 2, b.w - 4, b.h - 4, 10, c);
  lcd.drawRoundRect(b.x + 1, b.y + 1, b.w - 2, b.h - 2, 11, c);
}

// Short haptic click, given after the finger has lifted so the motor
// can't disturb the touch reading.
void clickFeedback() {
  if (!vibeActive) { drv.setRealtimeValue(0x50); tickUntil = millis() + 20; }
}

// ---------- Screens ----------
void drawList() {
  btnCount = 0;
  lcd.fillScreen(C_BG);

  // Header: big arrow targets, list name in the middle
  drawBtn(0, 0, 52, 40, "<", C_CARD, C_TEXT);
  drawBtn(188, 0, 52, 40, ">", C_CARD, C_TEXT);
  addBtn(0, 0, 52, 40, B_PREV);
  addBtn(188, 0, 52, 40, B_NEXTLIST);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.setTextDatum(middle_center);
  lcd.setFont(&fonts::Font2);
  String name = L().name;
  if (name.startsWith("Rife - ")) name = name.substring(7);
  if (name.length() > 16) name = name.substring(0, 15) + ".";
  lcd.drawString(name, 120, 13);
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(C_MUTED, C_BG);
  char sub[24];
  snprintf(sub, sizeof(sub), "list %d/%d", listIdx + 1, LIST_COUNT);
  lcd.drawString(sub, 120, 31);

  // Rows: full-width tap targets
  for (int r = 0; r < ROWS; r++) {
    int i = scroll + r, y = 42 + r * 37;
    if (i >= L().count) break;
    lcd.fillRoundRect(2, y + 1, 236, 35, 8, C_CARD);
    const Freq &f = L().items[i];
    lcd.setTextDatum(top_left);
    lcd.setFont(&fonts::Font2);
    lcd.setTextColor(C_ACCENT, C_CARD);
    String hz = hzText(f.hz) + " Hz";
    if (f.hz < VIBE_BELOW_HZ) hz += "  vibrate";
    lcd.drawString(hz, 12, y + 3);
    lcd.setFont(&fonts::Font0);
    lcd.setTextColor(C_MUTED, C_CARD);
    String lb = f.label;
    if (lb.length() > 36) lb = lb.substring(0, 35) + ".";
    lcd.drawString(lb, 12, y + 22);
    lcd.setTextDatum(middle_right);
    lcd.setFont(&fonts::Font2);
    lcd.setTextColor(C_TEXT, C_CARD);
    lcd.drawString(">", 230, y + 18);
    addBtn(0, y, 240, 37, (BtnId)(B_ROW0 + r));
  }

  // Footer: four equal buttons, 50 px tall, reaching the bottom edge
  bool canUp = scroll > 0, canDn = scroll + ROWS < L().count;
  char t[8];
  uint16_t d = DURATIONS[durIdx];
  snprintf(t, sizeof(t), "%dm", d / 60);
  drawBtn(0, 190, 60, 50, "UP", C_CARD, C_TEXT, canUp);
  drawBtn(60, 190, 60, 50, "DN", C_CARD, C_TEXT, canDn);
  drawBtn(120, 190, 60, 50, t, C_CARD, C_TEXT);
  drawBtn(180, 190, 60, 50, "ALL", C_ACCENT, 0x0000);
  addBtn(0, 190, 60, 50, B_UP, canUp);
  addBtn(60, 190, 60, 50, B_DOWN, canDn);
  addBtn(120, 190, 60, 50, B_TIME);
  addBtn(180, 190, 60, 50, B_ALL);
}

void drawPlayerStatic() {
  btnCount = 0;
  const Freq &f = L().items[curItem];
  lcd.fillScreen(C_BG);
  lcd.setTextColor(C_MUTED, C_BG);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_left);
  char top[32];
  if (playAll) snprintf(top, sizeof(top), "%d of %d", curItem + 1, L().count);
  else snprintf(top, sizeof(top), "Single tone");
  lcd.drawString(top, 8, 4);
  lcd.setTextDatum(top_right);
  char bat[12];
  snprintf(bat, sizeof(bat), "%d%%%s", pmu.getBatteryPercent(), pmu.isCharging() ? "+" : "");
  lcd.drawString(bat, 232, 4);

  // big number
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.setFont(&fonts::Font7);
  String hz = hzText(f.hz);
  lcd.setTextSize(hz.length() > 4 ? 0.8f : 1.0f);
  lcd.drawString(hz, 120, 56);
  lcd.setTextSize(1);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(C_MUTED, C_BG);
  const char *how = f.hz < VIBE_BELOW_HZ ? "Hz - vibration" : (f.hz > 18000 ? "Hz - above most speakers" : "Hz - speaker");
  lcd.drawString(how, 120, 90);

  // label (up to two lines)
  lcd.setTextColor(C_TEXT, C_BG);
  String lb = f.label, l1 = lb, l2 = "";
  if (lb.length() > 26) {
    int cut = lb.lastIndexOf(',', 26);
    if (cut < 0) cut = 26;
    l1 = lb.substring(0, cut + 1);
    l2 = lb.substring(cut + 1);
    l2.trim();
    if (l2.length() > 28) l2 = l2.substring(0, 27) + ".";
  }
  lcd.drawString(l1, 120, 110);
  if (l2.length()) lcd.drawString(l2, 120, 127);

  // volume bar
  lcd.setFont(&fonts::Font0);
  lcd.setTextDatum(middle_left);
  lcd.setTextColor(C_MUTED, C_BG);
  lcd.drawString("VOL", 10, 178);
  for (int i = 0; i < 10; i++)
    lcd.fillRoundRect(34 + i * 20, 174, 16, 8, 3, i < volume ? C_ACCENT : C_LINE);

  // buttons: 3 across for a single tone, 4 across when playing a list
  int n = playAll ? 4 : 3, w = 240 / n;
  drawBtn(0, 190, w, 50, "STOP", C_STOP, C_TEXT);
  drawBtn(w, 190, w, 50, "VOL -", C_CARD, C_TEXT, volume > 1);
  drawBtn(2 * w, 190, w, 50, "VOL +", C_CARD, C_TEXT, volume < 10);
  addBtn(0, 190, w, 50, B_STOP);
  addBtn(w, 190, w, 50, B_VOLDN, volume > 1);
  addBtn(2 * w, 190, w, 50, B_VOLUP, volume < 10);
  if (playAll) {
    drawBtn(3 * w, 190, w, 50, "NEXT", C_CARD, C_TEXT);
    addBtn(3 * w, 190, w, 50, B_SKIP);
  }
  lastSecShown = -1;
}

void drawPlayerProgress() {
  uint32_t el = millis() - startMs;
  int left = (int)((durMs > el ? durMs - el : 0) / 1000);
  if (left == lastSecShown) return;
  lastSecShown = left;
  int w = durMs ? (int)(220.0f * el / durMs) : 0;
  if (w > 220) w = 220;
  lcd.fillRoundRect(10, 144, 220, 8, 4, C_LINE);
  if (w > 0) lcd.fillRoundRect(10, 144, w, 8, 4, C_ACCENT);
  char t[12];
  snprintf(t, sizeof(t), "%d:%02d left", left / 60, left % 60);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(C_MUTED, C_BG);
  lcd.fillRect(60, 154, 120, 16, C_BG);
  lcd.drawString(t, 120, 154);
}

void drawTouchTest() {
  btnCount = 0;
  lcd.fillScreen(C_BG);
  for (int g = 40; g < 240; g += 40) {
    lcd.drawFastHLine(0, g, 240, C_LINE);
    lcd.drawFastVLine(g, 0, 240, C_LINE);
  }
  lcd.setTextDatum(middle_center);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.drawString("Check alignment", 120, 60);
  lcd.setTextColor(C_MUTED, C_BG);
  lcd.drawString("The dot should be under", 120, 82);
  lcd.drawString("your finger. Side button = done", 120, 100);
}

void touchTestPoint(int16_t x, int16_t y) {
  lcd.fillCircle(x, y, 6, C_ACCENT);
  char t[24];
  snprintf(t, sizeof(t), " x=%d  y=%d ", x, y);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(C_TEXT, C_STOP);
  lcd.drawString(t, 120, 140);
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
const Btn *findBtn(BtnId id) {
  for (uint8_t i = 0; i < btnCount; i++) if (btns[i].id == id) return &btns[i];
  return nullptr;
}

void activate(BtnId id) {
  clickFeedback();
  switch (id) {
    case B_PREV:     listIdx = (listIdx + LIST_COUNT - 1) % LIST_COUNT; scroll = 0; drawList(); break;
    case B_NEXTLIST: listIdx = (listIdx + 1) % LIST_COUNT; scroll = 0; drawList(); break;
    case B_ROW0: case B_ROW1: case B_ROW2: case B_ROW3:
      playAll = false; startItem(scroll + (id - B_ROW0)); break;
    case B_UP:   scroll = scroll >= ROWS ? scroll - ROWS : 0; drawList(); break;
    case B_DOWN: if (scroll + ROWS < L().count) scroll += ROWS; drawList(); break;
    case B_TIME: durIdx = (durIdx + 1) % (sizeof(DURATIONS) / sizeof(DURATIONS[0])); saveSettings(); drawList(); break;
    case B_ALL:  playAll = true; startItem(0); break;
    case B_STOP: stopPlayback(); break;
    case B_VOLDN: case B_VOLUP:
      if (id == B_VOLDN && volume > 1) volume--;
      if (id == B_VOLUP && volume < 10) volume++;
      saveSettings();
      if (g_tone) g_amp = volumeToAmp();
      if (vibeActive && vibeOn) vibeWrite(true);
      drawPlayerStatic();
      drawPlayerProgress();
      break;
    case B_SKIP:
      if (curItem + 1 < L().count) startItem(curItem + 1); else stopPlayback();
      break;
    default: break;
  }
}

// ---------- Touch calibration ----------
// Maps raw touch-chip coordinates to screen pixels with an affine transform
// worked out from three targets the user touches. This corrects any flip,
// swap, offset or scale between the touch layer and the display.
static float cal[6] = {1, 0, 0, 0, 1, 0};   // x = c0*rx + c1*ry + c2, y = c3*rx + c4*ry + c5

void mapTouch(int16_t rx, int16_t ry, int16_t &x, int16_t &y) {
  float fx = cal[0] * rx + cal[1] * ry + cal[2];
  float fy = cal[3] * rx + cal[4] * ry + cal[5];
  x = constrain((int)lroundf(fx), 0, 239);
  y = constrain((int)lroundf(fy), 0, 239);
}

// Wait for the finger to be off the glass, then for a steady 0.4 s touch.
bool waitHold(int16_t &rx, int16_t &ry, uint32_t timeoutMs) {
  uint32_t t0 = millis(), lastSeen = millis();
  int16_t x, y;
  while (millis() - t0 < timeoutMs) {                 // wait for release
    if (readTouch(x, y)) lastSeen = millis();
    else if (millis() - lastSeen > 250) break;
    delay(10);
  }
  long sx = 0, sy = 0; int n = 0;
  uint32_t holdStart = 0; lastSeen = 0;
  while (millis() - t0 < timeoutMs) {
    if (readTouch(x, y)) {
      if (!holdStart) { holdStart = millis(); sx = sy = 0; n = 0; }
      lastSeen = millis();
      sx += x; sy += y; n++;
      if (millis() - holdStart >= 400 && n >= 5) { rx = sx / n; ry = sy / n; return true; }
    } else if (holdStart && millis() - lastSeen > 150) {
      holdStart = 0;
    }
    delay(10);
  }
  return false;
}

void drawTarget(int tx, int ty, int step) {
  lcd.fillScreen(C_BG);
  lcd.setTextDatum(middle_center);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.drawString("Touch alignment", 120, 100);
  lcd.setTextColor(C_MUTED, C_BG);
  char t[32];
  snprintf(t, sizeof(t), "Press and hold the +  (%d/3)", step);
  lcd.drawString(t, 120, 122);
  lcd.drawCircle(tx, ty, 14, C_ACCENT);
  lcd.drawFastHLine(tx - 20, ty, 41, C_ACCENT);
  lcd.drawFastVLine(tx, ty - 20, 41, C_ACCENT);
}

void buzz(uint16_t ms) {
  drv.setRealtimeValue(0x60); delay(ms); drv.setRealtimeValue(0);
}

bool runCalibration() {
  const int TX[3] = {40, 200, 40}, TY[3] = {40, 40, 200};
  int16_t RX[3], RY[3];
  for (int i = 0; i < 3; i++) {
    drawTarget(TX[i], TY[i], i + 1);
    if (!waitHold(RX[i], RY[i], 30000)) goto fail;
    buzz(30);
  }
  {
    // Solve [rx ry 1] * [c0 c1 c2]' = tx (and the same for y) with Cramer's rule.
    double m[3][3], det;
    for (int i = 0; i < 3; i++) { m[i][0] = RX[i]; m[i][1] = RY[i]; m[i][2] = 1; }
    det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (fabs(det) < 100) goto fail;              // targets weren't distinct
    float c[6];
    for (int k = 0; k < 2; k++) {
      const int *T = k == 0 ? TX : TY;
      for (int col = 0; col < 3; col++) {
        double mm[3][3];
        for (int r = 0; r < 3; r++) for (int cc = 0; cc < 3; cc++) mm[r][cc] = (cc == col) ? T[r] : m[r][cc];
        double d = mm[0][0] * (mm[1][1] * mm[2][2] - mm[1][2] * mm[2][1])
                 - mm[0][1] * (mm[1][0] * mm[2][2] - mm[1][2] * mm[2][0])
                 + mm[0][2] * (mm[1][0] * mm[2][1] - mm[1][1] * mm[2][0]);
        c[k * 3 + col] = d / det;
      }
    }
    float scale = fabsf(c[0] * c[4] - c[1] * c[3]);   // area scale of the mapping
    if (scale < 0.3f || scale > 3.0f) goto fail;
    memcpy(cal, c, sizeof(cal));
    prefs.putBytes("cal", cal, sizeof(cal));
    return true;
  }
fail:
  lcd.fillScreen(C_BG);
  lcd.setTextDatum(middle_center);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(C_STOP, C_BG);
  lcd.drawString("Alignment didn't work", 120, 100);
  lcd.setTextColor(C_MUTED, C_BG);
  lcd.drawString("Hold side button 2 s to retry", 120, 124);
  delay(3000);
  return false;
}

// ---------- Touch handling ----------
// A button fires only after it has been held for HOLD_MS (a bar fills to
// show progress), and only once per press: the finger must lift before
// anything else can be pressed. Touch drop-outs under 120 ms are ignored.
static const uint32_t HOLD_MS = 500;
static bool fingerDown = false, fired = false;
static BtnId pressedId = B_NONE;
static int16_t lastX = 0, lastY = 0;
static uint32_t lastRawTouch = 0, pressStart = 0;
static int lastBar = -1;

void cancelPress() {
  const Btn *b = findBtn(pressedId);
  if (b) {
    highlight(*b, false);
    lcd.fillRect(b->x + 10, b->y + b->h - 6, b->w - 20, 3, b->id >= B_ROW0 && b->id <= B_ROW3 ? C_CARD
                 : (b->id == B_ALL ? C_ACCENT : (b->id == B_STOP ? C_STOP : C_CARD)));
  }
  pressedId = B_NONE;
}

void handleTouch(uint32_t now) {
  int16_t rx, ry;
  if (readTouch(rx, ry)) {
    lastRawTouch = now;
    mapTouch(rx, ry, lastX, lastY);
  }
  bool down = lastRawTouch && (now - lastRawTouch < 120);

  if (down && !fingerDown) {                 // finger went down
    fingerDown = true;
    fired = false;
    lastActivity = now;
    pressedId = B_NONE;
    if (!screenOn) { setBacklight(true); fired = true; return; }   // wake only
    if (screen == TOUCHTEST) { touchTestPoint(lastX, lastY); return; }
    const Btn *b = hitTest(lastX, lastY);
    if (b && b->enabled) {
      pressedId = b->id;
      pressStart = now;
      lastBar = -1;
      highlight(*b, true);
    }
  } else if (down) {                         // finger held
    lastActivity = now;
    if (screen == TOUCHTEST) { touchTestPoint(lastX, lastY); return; }
    if (pressedId == B_NONE || fired) return;
    const Btn *b = findBtn(pressedId);
    const Btn *at = hitTest(lastX, lastY);
    if (!b || !at || at->id != pressedId) { cancelPress(); return; }   // slid off
    uint32_t held = now - pressStart;
    int bar = (int)((b->w - 20) * min(held, HOLD_MS) / HOLD_MS);
    if (bar != lastBar) {
      lastBar = bar;
      lcd.fillRect(b->x + 10, b->y + b->h - 6, bar, 3, C_TEXT);
    }
    if (held >= HOLD_MS) {
      fired = true;
      BtnId id = pressedId;
      pressedId = B_NONE;
      activate(id);
    }
  } else if (fingerDown) {                   // finger lifted
    fingerDown = false;
    if (pressedId != B_NONE && !fired) cancelPress();   // let go too early
    pressedId = B_NONE;
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
  // Display + touch. The touch chip has no reset pin, so if earlier firmware
  // put it to sleep it stays asleep. Cutting its power for a moment resets it.
  pmu.disableALDO3();
  delay(300);
  pmu.setALDO3Voltage(3300); pmu.enableALDO3();
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
  pmu.enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ);
}

void setup() {
  Serial.begin(115200);
  // WiFi and Bluetooth are never initialised in this firmware.

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  initPower();
  delay(50);

  Wire1.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL);
  delay(300);                       // touch chip needs time after power-up
  bool touchOk = touchPresent();
  if (!touchOk) {                   // one more power cycle, then re-check
    pmu.disableALDO3(); delay(500); pmu.enableALDO3(); delay(500);
    touchOk = touchPresent();
  }

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
  bool haveCal = prefs.getBytesLength("cal") == sizeof(cal) &&
                 prefs.getBytes("cal", cal, sizeof(cal)) == sizeof(cal);
  if (!touchOk) {
    lcd.fillScreen(C_BG);
    lcd.setTextColor(C_STOP, C_BG);
    lcd.setTextDatum(middle_center);
    lcd.setFont(&fonts::Font2);
    lcd.drawString("Touch screen not detected", 120, 100);
    lcd.setTextColor(C_MUTED, C_BG);
    lcd.drawString("Hold side button 6 s, then", 120, 130);
    lcd.drawString("unplug USB and power on again", 120, 150);
    delay(5000);
  }
  if (touchOk && !haveCal && runCalibration()) {
    screen = TOUCHTEST;                     // let the user check the result
    drawTouchTest();
  } else {
    drawList();
  }
  lastActivity = millis();
}

void loop() {
  static uint32_t lastPmuPoll = 0;
  uint32_t now = millis();

  handleTouch(now);

  // end of the short haptic "click"
  if (tickUntil && (int32_t)(now - tickUntil) >= 0) {
    tickUntil = 0;
    if (!vibeActive) drv.setRealtimeValue(0);
  }

  // side button: short press = screen on/off (or leave the alignment check),
  // hold ~2 s = redo touch alignment, hold 6 s = power off
  if (now - lastPmuPoll > 100) {
    lastPmuPoll = now;
    pmu.getIrqStatus();
    if (pmu.isPekeyShortPressIrq()) {
      lastActivity = now;
      if (screen == TOUCHTEST) { screen = LIST; drawList(); setBacklight(true); }
      else setBacklight(!screenOn);
    }
    if (pmu.isPekeyLongPressIrq() && screen != PLAYER) {
      setBacklight(true);
      if (runCalibration()) { screen = TOUCHTEST; drawTouchTest(); }
      else { screen = LIST; drawList(); }
      lastActivity = millis();
      fingerDown = false; pressedId = B_NONE; lastRawTouch = 0;
      pmu.clearIrqStatus();
    }
    pmu.clearIrqStatus();
  }

  if (screenOn && screen != TOUCHTEST && (int32_t)(millis() - lastActivity) > SCREEN_TIMEOUT) setBacklight(false);

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
    if (millis() - startMs >= durMs) {     // fresh time: startItem() may have just run
      if (playAll && curItem + 1 < L().count) startItem(curItem + 1);
      else stopPlayback();
    } else if (screenOn) {
      drawPlayerProgress();
    }
  }

  delay(vibeActive ? 1 : 10);
}
