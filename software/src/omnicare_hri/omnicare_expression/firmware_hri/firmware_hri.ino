// led_control_fw.ino
//
// OmniCare LED firmware for WS2812B (NeoPixel) + Active Buzzer + Star Wars "Anthem" (Arduino Mega 2560)
//
// Serial protocol (single ASCII byte sent from the host over USB-Serial):
//   'B' -> Blue   (idle)   : soft breathing
//   'Y' -> Yellow (moving) : soft breathing
//   'R' -> Red    (error)  : fast blink
//   'X' -> One-shot: play anthem once (non-blocking)
//
// Notes:
// - This sketch drives a WS2812B LED strip using the Adafruit_NeoPixel library.
// - The buzzer is an ACTIVE buzzer, so driving BUZZER_PIN HIGH produces sound.
// - A periodic "heartbeat" beep runs only when the anthem is NOT playing.
// - While the anthem is playing, LEDs are frozen to avoid strip.show() jitter
//   and to preserve audio timing.

#include <Adafruit_NeoPixel.h>
#include <math.h>

// ------------------- Pin / device configuration -------------------
// WS2812B data pin + LED strip configuration
#define LED_PIN     6        // WS2812B data line
#define NUM_LEDS    20       // number of pixels in the strip
#define BRIGHTNESS  255      // global brightness (capped in show())

// Active buzzer control pin (HIGH = sound)
#define BUZZER_PIN  3        // ACTIVE buzzer (not a passive piezo)

// ------------------- LED driver instance --------------------------
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- Colors (RGB) ------------------------------------
// Base colors used in patterns; intensity is applied later via gamma scaling
const uint8_t BLUE_RGB[3]   = {  0,   0, 255};
const uint8_t YELLOW_RGB[3] = {255, 180,   0};
const uint8_t GREEN_RGB[3]  = {  0, 255,   0};
const uint8_t RED_RGB[3]    = {255,   0,   0};

// --------------- Breathing (B/Y) configuration --------------------
// Breathing = sine-mapped intensity between MIN..MAX at BREATH_FREQ_HZ
const float   BREATH_FREQ_HZ = 0.9f;   // pleasant/slow breathing rate
const uint8_t BREATH_MIN     = 100;    // lower bound intensity
const uint8_t BREATH_MAX     = 220;    // upper bound intensity

// --------------- Red blink (R) configuration ----------------------
const uint32_t RED_BLINK_MS  = 500;    // on/off toggle period (fast attention)

// ------------------ Periodic buzzer “heartbeat” -------------------
// Independent of LED mode; disabled while anthem is playing
const uint16_t BUZZ_ON_MS     = 500;   // beep ON window
const uint32_t BUZZ_PERIOD_MS = 5000;  // repetition period

bool          enableBuzzer = true;           // flag to enable/disable buzzer
bool          buzzOn = false;                 // buzzer is currently on?
bool          demo = false;
unsigned long buzzLastPeriodStart = 0;        // last beep cycle start
unsigned long buzzOnStart         = 0;        // time when current beep turned on

// --------------- LED mode state -----------------------------------
// current = 'B'|'Y'|'R' selected by serial; red_on for blink toggling
char current = 'B';             // default at boot: idle/blue
bool red_on  = true;            // current phase of red blink
unsigned long last_toggle = 0;  // last time red blink toggled

// ---- LED update throttle (reduce show() rate / avoid audio noise) ----
// WS2812B updates can be costly; cap to ~40 Hz to smooth visuals and avoid jitter
unsigned long next_led_update_ms = 0;
const unsigned long LED_UPDATE_PERIOD_MS = 25; // ~40 Hz cap

// ----------------- LED Helpers ------------------------------------
// Set the entire strip to a solid color and push to hardware
inline void setAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, r, g, b);
  strip.show();
}

// Turn all LEDs off quickly
inline void setAllOff() { setAll(0, 0, 0); }

// Simple gamma-like correction to make low intensities more perceptually smooth
inline uint8_t gamma8(uint8_t v) {
  uint16_t x = (uint16_t)v * (uint16_t)v;
  return (uint8_t)(x / 255);
}

// Apply intensity (0..255) with gamma to a base RGB and set all pixels
void setAllScaled(const uint8_t rgb[3], uint8_t intensity) {
  uint8_t gi = gamma8(intensity);
  uint16_t r = (uint16_t)rgb[0] * gi / 255;
  uint16_t g = (uint16_t)rgb[1] * gi / 255;
  uint16_t b = (uint16_t)rgb[2] * gi / 255;
  setAll((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

// Breathing animation: sine wave mapped to intensity window, non-blocking
void anim_breath(const uint8_t base_rgb[3]) {
  static unsigned long t0 = 0;
  if (t0 == 0) t0 = millis();                    // lazy init on first call
  float t = (millis() - t0) / 1000.0f;           // seconds since start
  float w = 2.0f * 3.1415926f * BREATH_FREQ_HZ * t;
  float s = (sinf(w) * 0.5f) + 0.5f;             // normalize to 0..1
  uint8_t intensity = (uint8_t)(BREATH_MIN + s * (BREATH_MAX - BREATH_MIN));
  setAllScaled(base_rgb, intensity);
}

// Heartbeat beep: turns buzzer on for BUZZ_ON_MS every BUZZ_PERIOD_MS
// Runs only when the anthem is not playing (see main loop)
void updateBuzzerPeriodic(unsigned long now) {
  if (!buzzOn) {
    // Wait for the next period to begin
    if (now - buzzLastPeriodStart >= BUZZ_PERIOD_MS) {
      digitalWrite(BUZZER_PIN, HIGH);    // ACTIVE buzzer: HIGH = ON
      buzzOn = true;
      buzzOnStart = now;
      buzzLastPeriodStart = now;
    }
  } else {
    // Turn off after ON window elapses
    if (now - buzzOnStart >= BUZZ_ON_MS) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzOn = false;
    }
  }
}

// ===================== ANTHEM (Star Wars) =====================
// Encapsulated as a struct to keep state and avoid Arduino prototype quirks.
struct Anthem {
  // ---- Phases (musical sections) ----
  enum Phase { IDLE = 0, S1A, S1B, S2A, S2B, VAR1, S2B_REPEAT, VAR2, DONE };

  // Note frequencies (Hz) for readability; suffix H = high octave
  static const unsigned int c_ = 261, d_ = 294, e_ = 329, f_ = 349, g_ = 391, gS_ = 415, a_ = 440, aS_ = 455, b_  = 466;
  static const unsigned int cH = 523, cSH = 554, dH = 587, dSH = 622, eH = 659, fH = 698, fSH = 740, gH = 784, gSH = 830, aH = 880;

  // Sections as arrays of {frequency, duration_ms}; names distinct from enum
  static const int NOTES_S1A[9][2];
  static const int NOTES_S1B[9][2];
  static const int NOTES_S2A[9][2];
  static const int NOTES_S2B[7][2];
  static const int NOTES_VAR1[8][2];
  static const int NOTES_VAR2[8][2];

  // Runtime state for the anthem finite-state machine
  static bool          playing;      // currently playing?
  static Phase         phase;        // current section
  static int           idx;          // index within current section
  static unsigned long startMs;      // start time (for max duration guard)
  static unsigned long noteEndMs;    // absolute time when current note ends
  static bool          inGap;        // true when in inter-note GAP

  // Gap between notes and safety cap on total anthem time
  static const unsigned long GAP_MS        = 50;
  static const unsigned long MAX_ANTHEM_MS = 15000;

  // Query
  static inline bool isPlaying() { return playing; }

  // Begin the anthem: reset state, silence buzzer, and “freeze” LEDs softly
  static void start() {
    playing = true;
    phase   = S1A;
    idx     = 0;
    startMs = millis();
    inGap   = false;
    noteEndMs = 0;
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);

    // Freeze LEDs to a calm color; avoid calling strip.show() repeatedly during audio
    setAllScaled(BLUE_RGB, 160);
  }

  // Stop immediately, clearing audio and state
  static void stop() {
    playing = false;
    phase   = DONE;
    idx     = 0;
    inGap   = false;
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Non-blocking update, called each loop() tick
  static void update(unsigned long now) {
    if (!playing) return;

    // Safety: stop on timeout or if marked DONE
    if ((now - startMs) >= MAX_ANTHEM_MS || phase == DONE) {
      // uint8_t = 0x22;
      Serial.write((uint8_t)0x22); // Anthem done! Sinalize to the node
      stop();
      return;
    }

    // If no note is currently scheduled, start or advance to the next one
    if (noteEndMs == 0) {
      int N = phaseLength(phase);
      if (idx >= N) {
        // End of section: advance phase; if no next, stop
        if (advancePhase(phase)) idx = 0; else { stop(); return; }
        N = phaseLength(phase);
      }
      int f, d;
      getNote(phase, idx, f, d);
      playNoteNow(f, d, now);
      return;
    }

    // Handle end of current note, with a short gap to avoid overlap artifacts
    if (!inGap) {
      if (now >= noteEndMs) {
        noTone(BUZZER_PIN);
        digitalWrite(BUZZER_PIN, LOW);
        noteEndMs = now + GAP_MS;
        inGap = true;
      }
    } else {
      // Gap finished -> move to next note
      if (now >= noteEndMs) {
        idx++;
        noteEndMs = 0;
        inGap = false;
      }
    }
  }

private:
  // Start a note now for dur_ms; if invalid freq/dur, just schedule a gap
  static void playNoteNow(int freq, int dur_ms, unsigned long now) {
    if (freq <= 0 || dur_ms <= 0) {
      noTone(BUZZER_PIN);
      digitalWrite(BUZZER_PIN, LOW);
      noteEndMs = now + GAP_MS;
      inGap = true;
      return;
    }
    tone(BUZZER_PIN, freq, dur_ms);      // ACTIVE buzzer will be driven with PWM tone
    noteEndMs = now + (unsigned long)dur_ms;
    inGap = false;
  }

  // Section sizes
  static int phaseLength(Phase p) {
    switch (p) {
      case S1A: return 9;
      case S1B: return 9;
      case S2A: return 9;
      case S2B: return 7;
      case VAR1: return 8;
      case S2B_REPEAT: return 7;
      case VAR2: return 8;
      default: return 0;
    }
  }

  // Advance to the next musical section; returns true if advanced
  static bool advancePhase(Phase &p) {
    switch (p) {
      case S1A: p = S1B; return true;
      case S1B: p = S2A; return true;
      case S2A: p = S2B; return true;
      case S2B: p = VAR1; return true;
      case VAR1: p = S2B_REPEAT; return true;
      case S2B_REPEAT: p = VAR2; return true;
      case VAR2: p = DONE; return true;
      default: return false;
    }
  }

  // Retrieve frequency/duration for the i-th note of section p
  static void getNote(Phase p, int i, int &freq, int &dur) {
    switch (p) {
      case S1A:        freq = NOTES_S1A[i][0];  dur = NOTES_S1A[i][1];  break;
      case S1B:        freq = NOTES_S1B[i][0];  dur = NOTES_S1B[i][1];  break;
      case S2A:        freq = NOTES_S2A[i][0];  dur = NOTES_S2A[i][1];  break;
      case S2B:        freq = NOTES_S2B[i][0];  dur = NOTES_S2B[i][1];  break;
      case VAR1:       freq = NOTES_VAR1[i][0]; dur = NOTES_VAR1[i][1]; break;
      case S2B_REPEAT: freq = NOTES_S2B[i][0];  dur = NOTES_S2B[i][1];  break;
      case VAR2:       freq = NOTES_VAR2[i][0]; dur = NOTES_VAR2[i][1]; break;
      default:         freq = 0; dur = 0; break;
    }
  }
};

// ---------- Static section definitions (freq, ms) ----------
const int Anthem::NOTES_S1A[9][2]  = {{a_,500},{a_,500},{a_,500},{f_,350},{cH,150},{a_,500},{f_,350},{cH,150},{a_,650}};
const int Anthem::NOTES_S1B[9][2]  = {{eH,500},{eH,500},{eH,500},{fH,350},{cH,150},{gS_,500},{f_,350},{cH,150},{a_,650}};
const int Anthem::NOTES_S2A[9][2]  = {{aH,500},{a_,300},{a_,150},{aH,500},{gSH,325},{gH,175},{fSH,125},{fH,125},{fSH,250}};
const int Anthem::NOTES_S2B[7][2]  = {{aS_,250},{dSH,500},{dH,325},{cSH,175},{cH,125},{b_,125},{cH,250}};
const int Anthem::NOTES_VAR1[8][2] = {{f_,250},{gS_,500},{f_,350},{a_,125},{cH,500},{a_,375},{cH,125},{eH,650}};
const int Anthem::NOTES_VAR2[8][2] = {{f_,250},{gS_,500},{f_,350},{cH,125},{a_,500},{f_,375},{cH,125},{a_,650}};

bool          Anthem::playing   = false;
Anthem::Phase Anthem::phase     = Anthem::IDLE;
int           Anthem::idx       = 0;
unsigned long Anthem::startMs   = 0;
unsigned long Anthem::noteEndMs = 0;
bool          Anthem::inGap     = false;

// ===================== SETUP/LOOP =====================
void setup() {
  // ---- LED init ----
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  setAllOff();                         // clear any residual state

  // ---- Serial link ----
  Serial.begin(115200);                 // host should match this baudrate

  // ---- Initial visual state ----
  anim_breath(BLUE_RGB);                // default: idle breathing
  last_toggle = millis();               // for red blink timing

  // ---- Buzzer init ----
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);        // ensure silent at boot
  buzzOn = false;
  buzzLastPeriodStart = millis();       // start the heartbeat period
}

void loop() {
  unsigned long now = millis();

  // -------- Serial handler --------
  // Pull the latest command; single letters are idempotent state setters
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'B' || c == 'Y' || c == 'R') {
      // Set the current LED mode
      current = c;
      if (current == 'R') {
        // On entering red mode, show solid red immediately (blink will start)
        red_on = true;
        last_toggle = now;
        setAll(RED_RGB[0], RED_RGB[1], RED_RGB[2]);
      }
    } else if (c == 'E') {
      // Enable buzzer periodic heartbeat
      enableBuzzer = true;
    } else if (c == 'D') {
      // Disable buzzer periodic heartbeat
      enableBuzzer = false;
      // Also turn off if currently buzzing
      if (buzzOn) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzOn = false;
      }
    } else if (c == 'X' && enableBuzzer) {
      // Trigger anthem one-shot (non-blocking)
      Anthem::start();
      demo = true;
      delay(1500);
    }
  }

  // -------- LEDs --------
  // LED animations run only if anthem is NOT playing (LEDs are frozen during anthem)
  if (!Anthem::isPlaying()) {
    // Throttle visual updates to ~40 Hz
    if (now >= next_led_update_ms) {
      if (current == 'B') {
        anim_breath(BLUE_RGB);
      } else if (current == 'Y') {
        anim_breath(YELLOW_RGB);
      }
      else if (current == 'G'){
        anim_breath(GREEN_RGB);   // success: breathing green
      }
      else { // 'R'
        // Fast attention blink using a simple toggle at RED_BLINK_MS
        if (now - last_toggle >= RED_BLINK_MS) {
          red_on = !red_on;
          last_toggle = now;
          if (red_on) setAll(RED_RGB[0], RED_RGB[1], RED_RGB[2]);
          else        setAllOff();
        }
      }
      next_led_update_ms = now + LED_UPDATE_PERIOD_MS;
    }
  }

  // -------- Periodic buzzer heartbeat (only when NOT playing the anthem) --------
  if (enableBuzzer) {
    if (!Anthem::isPlaying() && !demo) {
      updateBuzzerPeriodic(now);
    }
  
    // During the anthem, LEDs remain constant at the color set by Anthem::start()
    // -------- Anthem engine (drives buzzer tones while active) --------
    Anthem::update(now);
  }
}
