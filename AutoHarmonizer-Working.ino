// Teensy 4.0 + Audio Shield HP out, I2S MEMS mic IN, SSD1306 OLED centered UI
// Mic:  SD->8, BCLK(SCK)->21, LRCLK(WS)->20, VDD->3.3V, GND->GND, L/R->GND (Left slot)
// OLED: SDA->18, SCL->19, VCC->3.3V, GND->GND (I2C addr 0x3C)
// Encoder: A->14, B->15, C->GND  (we use INPUT_PULLUP on A/B)

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ---------- OLED ----------
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ---------- Audio graph ----------
AudioInputI2S            i2s_in;         // from MEMS mic
AudioEffectGranular      granular1;      // pitch shift
AudioOutputI2S           i2s_out;        // to SGTL5000 (Audio Shield)
AudioAnalyzeNoteFrequency notefreq;      // for Hz & note detection (input tap)
AudioControlSGTL5000     sgtl5000_1;

// Use LEFT input slot (L/R tied to GND). If tied high, change all "0" below to "1".
AudioConnection pc_in_to_gran(i2s_in, 0, granular1, 0);
AudioConnection pc_gran_to_L (granular1, 0, i2s_out, 0);
AudioConnection pc_gran_to_R (granular1, 0, i2s_out, 1);
AudioConnection pc_in_to_nf  (i2s_in, 0, notefreq, 0);

// Granular memory (~371 ms @ 44.1 kHz)
#define GRANULAR_SAMPLES 16384
int16_t granularMemory[GRANULAR_SAMPLES];

// ---------- Rotary encoder (polling) ----------
const int ENC_A = 14;
const int ENC_B = 15;
int semitones = 0;                // [-12..+12]
uint8_t encPrev = 0;
int encAcc = 0;
// Quadrature lookup table (4 edges per detent)
const int8_t ENC_TAB[16] = {
  0,-1,+1, 0,
 +1, 0, 0,-1,
 -1, 0, 0,+1,
  0,+1,-1, 0
};
inline uint8_t encRead2() {
  return (digitalReadFast(ENC_A) << 1) | digitalReadFast(ENC_B);
}

// ---------- Note helpers ----------
static const char* NOTE_NAMES[12] = {
  "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

bool midiToName(int n, char* out, size_t outLen) {
  if (n < 0 || n > 127) return false;
  int noteIndex = (n % 12 + 12) % 12;
  int octave    = (n / 12) - 1;
  snprintf(out, outLen, "%s%d", NOTE_NAMES[noteIndex], octave);
  return true;
}

bool hzToNearestMidi(float hz, int &midiOut) {
  if (hz <= 0.0f || !isfinite(hz)) return false;
  float nFloat = 69.0f + 12.0f * logf(hz / 440.0f) / logf(2.0f);
  midiOut = (int)lrintf(nFloat);
  if (midiOut < 0) midiOut = 0;
  if (midiOut > 127) midiOut = 127;
  return true;
}

void applyPitch() {
  // ratio = 2^(semitones/12)
  float ratio = powf(2.0f, semitones / 12.0f);
  granular1.setSpeed(ratio);
}

// Centered text helper
void drawCentered(const char* text, int y, int textSize) {
  int16_t x1, y1; uint16_t w, h;
  display.setTextSize(textSize);
  display.getTextBounds((char*)text, 0, 0, &x1, &y1, &w, &h);
  int x = (128 - (int)w) / 2;
  display.setCursor(x, y);
  display.print(text);
}

void setup() {
  // I2C first (shared by SGTL5000 + OLED)
  Wire.begin();

  // Audio engine + codec
  AudioMemory(30);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.65f);

  // Granular pitch shift setup
  granular1.begin(granularMemory, GRANULAR_SAMPLES);
  granular1.beginPitchShift(12.0f);   // grain length (ms). Tweak 8–25ms as you like.
  semitones = 0;
  applyPitch();

  // Pitch detector
  notefreq.begin(0.15f);

  // OLED last
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  encPrev = encRead2();
}

uint32_t lastDraw = 0;
// retain last valid input reading so display looks steady
float   lastHz = 0.0f;
int     lastMidi = -1;

void loop() {
  // --- Encoder polling (no interrupts) ---
  uint8_t encNow = encRead2();
  if (encNow != encPrev) {
    uint8_t idx = (encPrev << 2) | encNow;
    encPrev = encNow;
    encAcc -= ENC_TAB[idx];                 // accumulate edges
    while (encAcc >= 4) {                   // 4 edges per detent
      encAcc -= 4;
      if (semitones < 12) { semitones++; applyPitch(); }
    }
    while (encAcc <= -4) {
      encAcc += 4;
      if (semitones > -12) { semitones--; applyPitch(); }
    }
  }

  // --- Update input freq/note when available ---
  if (notefreq.available()) {
    float hz = notefreq.read();
    int midiN;
    if (hzToNearestMidi(hz, midiN)) {
      lastHz = hz;
      lastMidi = midiN;
    } else {
      lastHz = 0.0f;
      lastMidi = -1;
    }
  }

  // --- OLED layout: top (in freq + note), middle (±shift), bottom (output note) ---
  if (millis() - lastDraw >= 120) {
    lastDraw = millis();
    display.clearDisplay();

    // TOP: "<freq> Hz  <note>" centered
    char topLine[32];
    if (lastMidi >= 0 && lastHz > 0.0f) {
      char inName[8]; midiToName(lastMidi, inName, sizeof(inName));
      snprintf(topLine, sizeof(topLine), "%d Hz  %s", (int)(lastHz + 0.5f), inName);
    } else {
      snprintf(topLine, sizeof(topLine), "-- Hz  --");
    }
    drawCentered(topLine, 0, 1);

    // MIDDLE: "+N" or "-N" centered, large
    char shiftLine[8];
    snprintf(shiftLine, sizeof(shiftLine), "%+d", semitones);

    int16_t x1, y1; uint16_t w, h;
    display.setTextSize(3);
    display.getTextBounds(shiftLine, 0, 0, &x1, &y1, &w, &h);

    // Center vertically, then raise a tad (e.g. 3–4 pixels)
    int midY = (64 - (int)h) / 2 - 3;
    if (midY < 0) midY = 0;  // safety clamp
    drawCentered(shiftLine, midY, 3);

    // BOTTOM: output note name (input note + shift), centered and kept on-screen
    char bottomLine[16] = "--";
    if (lastMidi >= 0) {
      int outMidi = lastMidi + semitones;
      outMidi = constrain(outMidi, 0, 127);
      midiToName(outMidi, bottomLine, sizeof(bottomLine));
    }
    display.setTextSize(2);
    int16_t bx1, by1; uint16_t bw, bh;
    display.getTextBounds(bottomLine, 0, 0, &bx1, &by1, &bw, &bh);
    int by = 64 - (int)bh - 1;        // 1px above the bottom edge
    drawCentered(bottomLine, by, 2);

    display.display();
  }
}
