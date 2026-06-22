/*
  ============================================================================
   Smart Temperature Alert System  /  מערכת התרעת טמפרטורה חכמה
  ============================================================================
   Final project - Embedded Systems course / פרויקט גמר - מערכות משובצות
  ----------------------------------------------------------------------------
   EN: A temperature watchdog. An LM35DZ analog sensor measures temperature.
       Three LEDs (green / yellow / red) light up progressively as it gets
       hotter, and a buzzer beeps faster the hotter it becomes - an early
       warning against overheating.

   HE: שומר טמפרטורה. חיישן אנלוגי LM35DZ מודד את הטמפרטורה. שלוש נוריות
       (ירוק / צהוב / אדום) נדלקות בהדרגה ככל שמתחמם, וזמזם מצפצף מהר יותר
       ככל שחם יותר - התרעה מוקדמת מפני התחממות יתר.
  ----------------------------------------------------------------------------
   Design notes / הערות תכנון:
     * EN: Non-blocking design - no delay() in the main flow. Timing relies on
           millis() and on a Timer1 CTC hardware interrupt that schedules a new
           temperature sample every 250 ms.
     * HE: תכנון לא-חוסם - אין שימוש ב-delay() בזרימה הראשית. התזמון מבוסס על
           millis() ועל פסיקת חומרה של Timer1 במצב CTC, המתזמנת דגימת
           טמפרטורה חדשה כל 250 מילישניות.
  ----------------------------------------------------------------------------
   The LM35 sensor / חיישן ה-LM35:
     EN: The LM35 outputs an analog voltage of 10 mV per degree Celsius
         (e.g. 25 C -> 250 mV). With the default 5 V reference and a 10-bit
         ADC, one ADC step = 5000 mV / 1024 ~= 4.88 mV ~= 0.488 C.
     HE: ה-LM35 מוציא מתח אנלוגי של 10 מיליוולט לכל מעלה צלזיוס (למשל 25
         מעלות -> 250 מיליוולט). עם מתח ייחוס של 5 וולט וממיר 10 סיביות,
         צעד אחד = 5000/1024 ~= 4.88 מיליוולט ~= 0.488 מעלות.
  ----------------------------------------------------------------------------
   Wiring summary / סיכום חיווט (no transistor - kit has none / ללא טרנזיסטור):
     LM35DZ +Vs  -> 5V
     LM35DZ Vout -> A0   (analog input / כניסה אנלוגית)
     LM35DZ GND  -> GND
     Green  LED  -> D11 (+ 220R to LED -> GND)
     Yellow LED  -> D10 (+ 220R to LED -> GND)
     Red    LED  -> D9  (+ 220R to LED -> GND)
     Buzzer +    -> D6   ;  Buzzer -  -> GND   (driven directly / מופעל ישירות)
   ============================================================================
*/

// ===========================================================================
//  Buzzer type / סוג הזמזם
// ---------------------------------------------------------------------------
//  EN: Set to 1 for a PASSIVE buzzer (needs tone()), or 0 for an ACTIVE buzzer
//      (built-in oscillator - just switch it on/off with digitalWrite).
//      Not sure? A passive buzzer usually shows a green circuit board on its
//      underside; an active one is fully sealed. If unsure, leave it at 1.
//  HE: הגדירו 1 לזמזם פסיבי (דורש tone()), או 0 לזמזם אקטיבי (מתנד מובנה -
//      פשוט מדליקים/מכבים עם digitalWrite). לא בטוחים? זמזם פסיבי מראה בדרך
//      כלל לוח ירוק בתחתיתו; אקטיבי אטום לחלוטין. בספק - השאירו 1.
#define BUZZER_IS_PASSIVE 1

// ===========================================================================
//  Pin definitions / הגדרות רגליים
// ===========================================================================
const uint8_t LM35_PIN   = A0;  // EN: LM35 analog output / HE: יציאת LM35 האנלוגית
const uint8_t LED_GREEN  = 11;  // EN: green LED  / HE: נורית ירוקה
const uint8_t LED_YELLOW = 10;  // EN: yellow LED / HE: נורית צהובה
const uint8_t LED_RED    = 9;   // EN: red LED    / HE: נורית אדומה
const uint8_t BUZZER_PIN = 6;   // EN: buzzer (direct drive) / HE: זמזם (הפעלה ישירה)

// ===========================================================================
//  Temperature thresholds (Celsius) / ספי טמפרטורה (צלזיוס)
// ---------------------------------------------------------------------------
//  EN: Change these to suit your demo. Touching the sensor reaches ~32 C; a
//      lighter or hair dryer easily passes 40 C.
//  HE: שנו ערכים אלו לפי ההדגמה. מגע בחיישן מגיע לכ-32 מעלות; מצית או מייבש
//      שיער חוצים בקלות 40 מעלות.
// ===========================================================================
const float TEMP_WARM = 30.0;  // EN: above -> yellow on / HE: מעל - הצהובה נדלקת
const float TEMP_HOT  = 40.0;  // EN: above -> red on + fast beep / HE: מעל - אדומה + צפצוף מהיר

// ===========================================================================
//  Timing constants (milliseconds) / קבועי תזמון (מילישניות)
// ===========================================================================
const unsigned long BEEP_ON_MS    = 40;  // EN: length of a single beep / HE: אורך צפצוף בודד
const unsigned long FAST_BEEP_GAP = 70;  // EN: gap at the hottest end / HE: מרווח בקצה הכי חם
const unsigned long SLOW_BEEP_GAP = 500; // EN: gap at the warm edge / HE: מרווח בקצה החמים

// EN: Tone frequency used for a passive buzzer (Hz) / HE: תדר הצליל לזמזם פסיבי (הרץ)
const unsigned int BUZZER_FREQ = 3000;

// ===========================================================================
//  ADC / sensor constants / קבועי ממיר וחיישן
// ===========================================================================
const float ADC_REF_VOLTS = 5.0;   // EN: ADC reference voltage / HE: מתח ייחוס הממיר
const int   ADC_STEPS      = 1024;  // EN: 10-bit ADC resolution / HE: רזולוציית ממיר 10 סיביות
const int   SMOOTH_SAMPLES = 8;     // EN: samples to average per reading / HE: מספר דגימות למיצוע

// ===========================================================================
//  Volatile flag shared with the Timer1 ISR / דגל נדיף המשותף עם פסיקת Timer1
// ===========================================================================
// EN: 'volatile' tells the compiler this may change inside an interrupt.
// HE: המילה 'volatile' מודיעה למהדר שהמשתנה עשוי להשתנות בתוך פסיקה.
volatile bool sampleDue = false; // EN: time to read the sensor / HE: דגל - הגיע זמן לדגום

// ===========================================================================
//  Regular program state / מצב התוכנית הרגיל
// ===========================================================================
float currentTempC = 25.0;       // EN: latest temperature (C) / HE: הטמפרטורה האחרונה

unsigned long lastBeepEdge = 0;  // EN: millis() of last buzzer change / HE: זמן שינוי הזמזם
bool          buzzerIsOn   = false; // EN: current buzzer state / HE: מצב הזמזם הנוכחי


// ===========================================================================
//  ISR: Timer1 compare match  /  פסיקה: התאמת השוואה של Timer1
// ---------------------------------------------------------------------------
//  EN: Fires every 250 ms (configured in setupTimer1). It only raises a flag;
//      the slow analog reading is done in the main loop to keep the ISR short.
//  HE: מופעלת כל 250 מ"ש (מוגדר ב-setupTimer1). היא רק מרימה דגל; הקריאה
//      האנלוגית האיטית נעשית בלולאה הראשית כדי להשאיר את הפסיקה קצרה.
// ===========================================================================
ISR(TIMER1_COMPA_vect) {
  sampleDue = true;
}


// ===========================================================================
//  setupTimer1()  /  אתחול Timer1
// ---------------------------------------------------------------------------
//  EN: Configures Timer1 in CTC mode to interrupt every 250 ms. With a 16 MHz
//      clock and a 1024 prescaler the timer ticks at 15625 Hz, so
//      0.250 s * 15625 = 3906 ticks per period.
//  HE: מגדיר את Timer1 במצב CTC כדי לייצר פסיקה כל 250 מ"ש. בשעון 16 מגה-הרץ
//      ומחלק 1024, הטיימר פועם ב-15625 הרץ, ולכן 0.250 שנ' כפול 15625 = 3906.
// ===========================================================================
void setupTimer1() {
  noInterrupts();            // EN: disable interrupts while configuring / HE: כיבוי פסיקות בזמן ההגדרה
  TCCR1A = 0;                // EN: clear control register A / HE: איפוס רגיסטר בקרה A
  TCCR1B = 0;                // EN: clear control register B / HE: איפוס רגיסטר בקרה B
  TCNT1  = 0;                // EN: reset the counter / HE: איפוס המונה
  OCR1A  = 3906;             // EN: compare value for ~250 ms / HE: ערך השוואה ל-250 מ"ש
  TCCR1B |= (1 << WGM12);    // EN: CTC mode / HE: מצב CTC
  TCCR1B |= (1 << CS12) | (1 << CS10); // EN: prescaler = 1024 / HE: מחלק תדר 1024
  TIMSK1 |= (1 << OCIE1A);   // EN: enable compare-match A interrupt / HE: הפעלת פסיקת השוואה A
  interrupts();              // EN: re-enable interrupts / HE: החזרת הפסיקות
}


// ===========================================================================
//  readTemperature()  /  קריאת הטמפרטורה
// ---------------------------------------------------------------------------
//  EN: Reads the LM35 several times, averages the samples to reduce noise, and
//      converts the result to degrees Celsius. The LM35 gives 10 mV per C, so:
//          volts = adc * Vref / 1024
//          tempC = volts * 100
//  HE: קורא את ה-LM35 מספר פעמים, ממצע את הדגימות כדי להפחית רעש, וממיר את
//      התוצאה למעלות צלזיוס. ה-LM35 נותן 10 מיליוולט למעלה, ולכן:
//          מתח  = adc * Vref / 1024
//          טמפ' = מתח * 100
// ===========================================================================
float readTemperature() {
  long sum = 0;
  for (int i = 0; i < SMOOTH_SAMPLES; i++) {
    sum += analogRead(LM35_PIN);
  }
  float avgAdc = (float)sum / SMOOTH_SAMPLES;
  float volts  = avgAdc * ADC_REF_VOLTS / ADC_STEPS;
  return volts * 100.0; // EN: 10 mV per C -> *100 / HE: 10 מיליוולט למעלה -> כפול 100
}


// ===========================================================================
//  buzzerOn() / buzzerOff()  /  הדלקת וכיבוי הזמזם
// ---------------------------------------------------------------------------
//  EN: Small wrappers so the rest of the code does not care whether the buzzer
//      is passive (tone) or active (digitalWrite). Controlled by the
//      BUZZER_IS_PASSIVE switch at the top of the file.
//  HE: עטיפות קטנות כך ששאר הקוד לא צריך לדעת אם הזמזם פסיבי (tone) או אקטיבי
//      (digitalWrite). נשלט על ידי המתג BUZZER_IS_PASSIVE בראש הקובץ.
// ===========================================================================
void buzzerOn() {
#if BUZZER_IS_PASSIVE
  tone(BUZZER_PIN, BUZZER_FREQ);
#else
  digitalWrite(BUZZER_PIN, HIGH);
#endif
}

void buzzerOff() {
#if BUZZER_IS_PASSIVE
  noTone(BUZZER_PIN);
#else
  digitalWrite(BUZZER_PIN, LOW);
#endif
}


// ===========================================================================
//  updateLeds()  /  עדכון הנוריות
// ---------------------------------------------------------------------------
//  EN: Lights the LEDs according to the temperature zone:
//        < 30 C        : green only           (normal)
//        30 C - 40 C   : green + yellow        (warming)
//        > 40 C        : green + yellow + red  (overheating)
//  HE: מדליק את הנוריות לפי אזור הטמפרטורה:
//        מתחת ל-30     : ירוק בלבד            (תקין)
//        בין 30 ל-40   : ירוק + צהוב          (מתחמם)
//        מעל 40        : ירוק + צהוב + אדום    (התחממות יתר)
// ===========================================================================
void updateLeds() {
  bool green  = true;                       // EN: always on while powered / HE: דולקת תמיד
  bool yellow = (currentTempC >= TEMP_WARM);
  bool red    = (currentTempC >= TEMP_HOT);

  digitalWrite(LED_GREEN,  green  ? HIGH : LOW);
  digitalWrite(LED_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(LED_RED,    red    ? HIGH : LOW);
}


// ===========================================================================
//  beepGapForTemp()  /  חישוב מרווח הצפצוף לפי הטמפרטורה
// ---------------------------------------------------------------------------
//  EN: Maps the temperature to the silent gap between beeps. The hotter it is,
//      the shorter the gap (faster beeping). Returns -1 below TEMP_WARM, which
//      means "do not beep".
//  HE: ממפה את הטמפרטורה למרווח השקט שבין צפצופים. ככל שחם יותר, המרווח קצר
//      יותר (צפצוף מהיר יותר). מחזיר 1- מתחת ל-TEMP_WARM, כלומר "אין לצפצף".
// ===========================================================================
long beepGapForTemp() {
  if (currentTempC < TEMP_WARM) {
    return -1; // EN: cool enough, stay silent / HE: קריר מספיק, להישאר בשקט
  }

  // EN: map the warm..hot range to slow..fast gaps, using x10 fixed point so
  //     map() (which works on integers) keeps the fractional degrees.
  // HE: ממפים את הטווח חמים..חם למרווחים איטי..מהיר, בעזרת נקודה קבועה כפול 10
  //     כדי ש-map() (העובד על מספרים שלמים) ישמור את המעלות השבריות.
  long t10    = (long)(currentTempC * 10.0);
  long warm10 = (long)(TEMP_WARM   * 10.0);
  long hot10  = (long)(TEMP_HOT    * 10.0);

  long gap = map(t10, warm10, hot10, SLOW_BEEP_GAP, FAST_BEEP_GAP);
  gap = constrain(gap, (long)FAST_BEEP_GAP, (long)SLOW_BEEP_GAP);
  return gap;
}


// ===========================================================================
//  updateBuzzer()  /  עדכון הזמזם
// ---------------------------------------------------------------------------
//  EN: Non-blocking beeper. Uses millis() to toggle the buzzer on/off so the
//      beep rate matches the temperature - faster when hotter.
//  HE: זמזם לא-חוסם. משתמש ב-millis() כדי להחליף את מצב הזמזם, כך שקצב
//      הצפצוף תואם לטמפרטורה - מהיר יותר כשחם יותר.
// ===========================================================================
void updateBuzzer() {
  long gap = beepGapForTemp();

  // EN: below the warm threshold -> make sure the buzzer is silent.
  // HE: מתחת לסף החימום -> לוודא שהזמזם שקט.
  if (gap < 0) {
    if (buzzerIsOn) {
      buzzerOff();
      buzzerIsOn = false;
    }
    return;
  }

  unsigned long now = millis();

  if (buzzerIsOn) {
    // EN: turn the beep off after BEEP_ON_MS / HE: כיבוי הצפצוף לאחר BEEP_ON_MS
    if (now - lastBeepEdge >= BEEP_ON_MS) {
      buzzerOff();
      buzzerIsOn   = false;
      lastBeepEdge = now;
    }
  } else {
    // EN: start a new beep after the (temperature-based) gap.
    // HE: התחלת צפצוף חדש אחרי המרווח (התלוי בטמפרטורה).
    if (now - lastBeepEdge >= (unsigned long)gap) {
      buzzerOn();
      buzzerIsOn   = true;
      lastBeepEdge = now;
    }
  }
}


// ===========================================================================
//  setup()  /  אתחול ראשוני
// ---------------------------------------------------------------------------
//  EN: Runs once at power-up. Configures pin directions, Timer1 and the serial
//      port used for debugging.
//  HE: רץ פעם אחת בהפעלה. מגדיר את כיווני הרגליים, את Timer1 ואת היציאה
//      הטורית המשמשת לניפוי שגיאות.
// ===========================================================================
void setup() {
  Serial.begin(9600);

  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  // EN: LM35_PIN is analog input - no pinMode needed / HE: כניסה אנלוגית - אין צורך ב-pinMode

  // EN: start the periodic-sampling timer / HE: הפעלת טיימר הדגימה המחזורי
  setupTimer1();

  Serial.println(F("Smart Temperature Alert System - ready / המערכת מוכנה"));
}


// ===========================================================================
//  loop()  /  הלולאה הראשית
// ---------------------------------------------------------------------------
//  EN: The main loop never blocks. Each pass it: (1) reads a new temperature
//      sample if Timer1 asked for one, (2) refreshes the LEDs and the buzzer,
//      and (3) prints debug info. Everything is driven by a flag and millis(),
//      so the CPU is always responsive.
//  HE: הלולאה הראשית אינה חוסמת לעולם. בכל מעבר היא: (1) קוראת דגימת
//      טמפרטורה חדשה אם Timer1 ביקש זאת, (2) מרעננת את הנוריות והזמזם,
//      ו-(3) מדפיסה מידע לניפוי. הכול מונע על ידי דגל ו-millis(), כך שהמעבד
//      תמיד זמין.
// ===========================================================================
void loop() {
  // (1) EN: a new sample was scheduled by Timer1 / HE: דגימה חדשה תוזמנה ע"י Timer1
  if (sampleDue) {
    sampleDue = false;
    currentTempC = readTemperature();
  }

  // (2) EN: drive the outputs / HE: הפעלת הפלטים
  updateLeds();
  updateBuzzer();

  // (3) EN: lightweight debug print, throttled / HE: הדפסת ניפוי קלה, מוגבלת בקצב
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print(F("Temperature / טמפרטורה: "));
    Serial.print(currentTempC, 1);
    Serial.println(F(" C"));
  }
}
