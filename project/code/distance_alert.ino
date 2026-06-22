/*
  ============================================================================
   Smart Distance Alert System  /  מערכת התרעת מרחק חכמה
  ============================================================================
   Final project - Embedded Systems course / פרויקט גמר - מערכות משובצות
  ----------------------------------------------------------------------------
   EN: An ultrasonic parking-sensor style system. An HC-SR04 sensor measures
       the distance to an object. Three LEDs (green / yellow / red) light up
       progressively as the object gets closer, and a passive buzzer (driven
       through a BC547 NPN transistor) beeps faster the closer the object is.

   HE: מערכת בסגנון חיישן חניה. חיישן אולטרסוני HC-SR04 מודד את המרחק לעצם.
       שלוש נוריות (ירוק / צהוב / אדום) נדלקות בהדרגה ככל שהעצם מתקרב, וזמזם
       פסיבי (המופעל דרך טרנזיסטור NPN מסוג BC547) מצפצף מהר יותר ככל שהעצם
       קרוב יותר.
  ----------------------------------------------------------------------------
   Design notes / הערות תכנון:
     * EN: Non-blocking design - no delay() in the main flow. Timing is based
           on millis() and on two hardware mechanisms:
             1. Timer1 CTC interrupt schedules a new measurement every 60 ms.
             2. An external interrupt on the ECHO pin measures the echo pulse
                width without blocking the CPU (no pulseIn()).
     * HE: תכנון לא-חוסם - אין שימוש ב-delay() בזרימה הראשית. התזמון מבוסס על
           millis() ועל שני מנגנוני חומרה:
             1. פסיקת Timer1 במצב CTC מתזמנת מדידה חדשה כל 60 מילישניות.
             2. פסיקה חיצונית על רגל ה-ECHO מודדת את רוחב פולס ההד ללא חסימת
                המעבד (ללא pulseIn()).
  ----------------------------------------------------------------------------
   Wiring summary / סיכום חיווט:
     HC-SR04 TRIG -> D4
     HC-SR04 ECHO -> D2  (INT0 - interrupt capable / רגל הניתנת לפסיקה)
     Green  LED   -> D11 (+ 220R to LED -> GND)
     Yellow LED   -> D10 (+ 220R to LED -> GND)
     Red    LED   -> D9  (+ 220R to LED -> GND)
     Buzzer       -> D6  -> 1k -> BC547 Base ; Collector -> buzzer- ; Emitter -> GND
   ============================================================================
*/

// ===========================================================================
//  Pin definitions / הגדרות רגליים
// ===========================================================================
const uint8_t TRIG_PIN   = 4;   // EN: HC-SR04 trigger output  / HE: יציאת טריגר לחיישן
const uint8_t ECHO_PIN   = 2;   // EN: HC-SR04 echo input (INT0)/ HE: כניסת הד (פסיקה INT0)
const uint8_t LED_GREEN  = 11;  // EN: green LED  / HE: נורית ירוקה
const uint8_t LED_YELLOW = 10;  // EN: yellow LED / HE: נורית צהובה
const uint8_t LED_RED    = 9;   // EN: red LED    / HE: נורית אדומה
const uint8_t BUZZER_PIN = 6;   // EN: buzzer via NPN transistor / HE: זמזם דרך טרנזיסטור

// ===========================================================================
//  Distance thresholds (centimeters) / ספי מרחק (סנטימטרים)
// ===========================================================================
const int FAR_THRESHOLD  = 50;  // EN: above this -> green only / HE: מעל ערך זה - ירוק בלבד
const int NEAR_THRESHOLD = 20;  // EN: below this -> danger zone / HE: מתחת לערך זה - אזור סכנה
const int MAX_DISTANCE   = 400; // EN: sensor practical max (cm) / HE: מרחק מרבי מעשי של החיישן

// ===========================================================================
//  Timing constants (milliseconds) / קבועי תזמון (מילישניות)
// ===========================================================================
const unsigned long BEEP_ON_MS    = 30;  // EN: length of a single beep / HE: אורך צפצוף בודד
const unsigned long FAST_BEEP_GAP = 60;  // EN: gap when object is closest / HE: מרווח כשהעצם הכי קרוב
const unsigned long SLOW_BEEP_GAP = 400; // EN: gap at the far edge / HE: מרווח בקצה הרחוק

// EN: Buzzer tone frequency for the passive buzzer (Hz).
// HE: תדר הצליל לזמזם הפסיבי (הרץ).
const unsigned int BUZZER_FREQ = 3000;

// ===========================================================================
//  Volatile variables shared with the ISRs / משתנים נדיפים המשותפים עם הפסיקות
// ===========================================================================
// EN: 'volatile' tells the compiler these may change inside an interrupt,
//     so it must not cache them in a register.
// HE: המילה 'volatile' מודיעה למהדר שהמשתנים עשויים להשתנות בתוך פסיקה,
//     ולכן אסור לו לשמור אותם זמנית ברגיסטר.
volatile unsigned long echoStartUs = 0;     // EN: micros() at echo rising edge / HE: זמן עליית ההד
volatile unsigned long echoWidthUs = 0;     // EN: measured echo pulse width / HE: רוחב פולס ההד שנמדד
volatile bool          echoReady   = false; // EN: a fresh echo is available / HE: דגל - מדידה חדשה מוכנה
volatile bool          measureDue  = false; // EN: time to fire a new trigger / HE: דגל - הגיע זמן טריגר חדש

// ===========================================================================
//  Regular program state / מצב התוכנית הרגיל
// ===========================================================================
long currentDistance = MAX_DISTANCE; // EN: latest valid distance (cm) / HE: המרחק התקֵף האחרון

unsigned long lastBeepEdge = 0;      // EN: millis() of last buzzer state change / HE: זמן שינוי מצב הזמזם
bool          buzzerIsOn   = false;  // EN: current buzzer state / HE: מצב הזמזם הנוכחי


// ===========================================================================
//  ISR: Timer1 compare match  /  פסיקה: התאמת השוואה של Timer1
// ---------------------------------------------------------------------------
//  EN: Fires every 60 ms (configured in setupTimer1). It only raises a flag;
//      the heavy work is done in the main loop to keep the ISR short.
//  HE: מופעלת כל 60 מ"ש (מוגדר ב-setupTimer1). היא רק מרימה דגל; העבודה
//      הכבדה נעשית בלולאה הראשית כדי להשאיר את הפסיקה קצרה.
// ===========================================================================
ISR(TIMER1_COMPA_vect) {
  measureDue = true;
}


// ===========================================================================
//  ISR: ECHO pin external interrupt  /  פסיקה: רגל ECHO חיצונית
// ---------------------------------------------------------------------------
//  EN: Triggered on every change of the ECHO line. On the rising edge we store
//      the start time; on the falling edge we compute the pulse width. This
//      replaces the blocking pulseIn() call.
//  HE: מופעלת בכל שינוי במצב רגל ה-ECHO. בעליית האות שומרים את זמן ההתחלה;
//      בירידת האות מחשבים את רוחב הפולס. זה מחליף את הקריאה החוסמת pulseIn().
// ===========================================================================
void echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStartUs = micros();          // EN: rising edge / HE: עליית האות
  } else {
    echoWidthUs = micros() - echoStartUs; // EN: falling edge / HE: ירידת האות
    echoReady   = true;
  }
}


// ===========================================================================
//  setupTimer1()  /  אתחול Timer1
// ---------------------------------------------------------------------------
//  EN: Configures Timer1 in CTC mode to generate a compare-match interrupt
//      every 60 ms. With a 16 MHz clock and a 1024 prescaler the timer ticks
//      at 15625 Hz, so 0.060 s * 15625 = 937 ticks per period.
//  HE: מגדיר את Timer1 במצב CTC כדי לייצר פסיקת התאמה כל 60 מ"ש. בשעון של
//      16 מגה-הרץ ומחלק 1024, הטיימר פועם ב-15625 הרץ, ולכן 0.060 שנ' כפול
//      15625 = 937 פעימות לכל מחזור.
// ===========================================================================
void setupTimer1() {
  noInterrupts();            // EN: disable interrupts while configuring / HE: כיבוי פסיקות בזמן ההגדרה
  TCCR1A = 0;                // EN: clear control register A / HE: איפוס רגיסטר בקרה A
  TCCR1B = 0;                // EN: clear control register B / HE: איפוס רגיסטר בקרה B
  TCNT1  = 0;                // EN: reset the counter / HE: איפוס המונה
  OCR1A  = 937;             // EN: compare value for ~60 ms / HE: ערך השוואה ל-60 מ"ש בקירוב
  TCCR1B |= (1 << WGM12);    // EN: CTC mode (clear timer on compare) / HE: מצב CTC
  TCCR1B |= (1 << CS12) | (1 << CS10); // EN: prescaler = 1024 / HE: מחלק תדר 1024
  TIMSK1 |= (1 << OCIE1A);   // EN: enable compare-match A interrupt / HE: הפעלת פסיקת השוואה A
  interrupts();              // EN: re-enable interrupts / HE: החזרת הפסיקות
}


// ===========================================================================
//  startMeasurement()  /  התחלת מדידה
// ---------------------------------------------------------------------------
//  EN: Sends the 10 microsecond trigger pulse that asks the HC-SR04 to emit
//      an ultrasonic burst. The 10 us pulse is the only micro-delay we allow;
//      it is required by the sensor and is far too short to affect timing.
//  HE: שולח את פולס הטריגר באורך 10 מיקרו-שניות שמבקש מה-HC-SR04 לפלוט פרץ
//      אולטרסוני. פולס ה-10 מיקרו-שניות הוא ההשהיה הזעירה היחידה שאנו מתירים;
//      הוא נדרש על ידי החיישן וקצר מכדי להשפיע על התזמון.
// ===========================================================================
void startMeasurement() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}


// ===========================================================================
//  updateDistance()  /  עדכון המרחק
// ---------------------------------------------------------------------------
//  EN: If the echo ISR captured a new pulse width, convert it to centimeters.
//      Sound travels at ~343 m/s = 0.0343 cm/us; we divide by 2 for the round
//      trip. Readings outside the valid range are ignored.
//  HE: אם פסיקת ההד תפסה רוחב פולס חדש, ממירים אותו לסנטימטרים. הקול נע
//      במהירות של כ-343 מ'/שנ' = 0.0343 ס"מ/מיקרו-שנייה; מחלקים ב-2 בשל הדרך
//      הלוך ושוב. קריאות מחוץ לטווח התקֵף מתעלמים מהן.
// ===========================================================================
void updateDistance() {
  if (!echoReady) {
    return; // EN: nothing new yet / HE: אין עדיין מדידה חדשה
  }

  // EN: copy the volatile value atomically / HE: העתקה אטומית של הערך הנדיף
  noInterrupts();
  unsigned long widthUs = echoWidthUs;
  echoReady = false;
  interrupts();

  long distance = (long)(widthUs * 0.0343 / 2.0);

  // EN: keep only physically plausible readings / HE: שמירת קריאות סבירות פיזית בלבד
  if (distance >= 2 && distance <= MAX_DISTANCE) {
    currentDistance = distance;
  }
}


// ===========================================================================
//  updateLeds()  /  עדכון הנוריות
// ---------------------------------------------------------------------------
//  EN: Lights the LEDs according to the distance zone:
//        > 50 cm        : green only
//        20 cm - 50 cm  : green + yellow
//        < 20 cm        : green + yellow + red (all on)
//  HE: מדליק את הנוריות לפי אזור המרחק:
//        מעל 50 ס"מ        : ירוק בלבד
//        בין 20 ל-50 ס"מ   : ירוק + צהוב
//        מתחת ל-20 ס"מ     : ירוק + צהוב + אדום (הכול דולק)
// ===========================================================================
void updateLeds() {
  bool green  = true;                              // EN: always on while powered / HE: דולקת תמיד
  bool yellow = (currentDistance <= FAR_THRESHOLD);
  bool red    = (currentDistance <  NEAR_THRESHOLD);

  digitalWrite(LED_GREEN,  green  ? HIGH : LOW);
  digitalWrite(LED_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(LED_RED,    red    ? HIGH : LOW);
}


// ===========================================================================
//  beepGapForDistance()  /  חישוב מרווח הצפצוף לפי מרחק
// ---------------------------------------------------------------------------
//  EN: Maps the distance to the silent gap between beeps. The closer the
//      object, the shorter the gap (faster beeping). Returns -1 when the
//      object is farther than FAR_THRESHOLD, meaning "do not beep".
//  HE: ממפה את המרחק למרווח השקט שבין צפצופים. ככל שהעצם קרוב יותר, המרווח
//      קצר יותר (צפצוף מהיר יותר). מחזיר 1- כשהעצם רחוק מ-FAR_THRESHOLD,
//      כלומר "אין לצפצף".
// ===========================================================================
long beepGapForDistance() {
  if (currentDistance > FAR_THRESHOLD) {
    return -1; // EN: too far, stay silent / HE: רחוק מדי, להישאר בשקט
  }
  // EN: linear map from [0..FAR] cm to [FAST..SLOW] ms, then clamp.
  // HE: מיפוי ליניארי מ-[0..FAR] ס"מ ל-[FAST..SLOW] מ"ש, ואז הגבלה לטווח.
  long gap = map(currentDistance, NEAR_THRESHOLD, FAR_THRESHOLD,
                 FAST_BEEP_GAP, SLOW_BEEP_GAP);
  gap = constrain(gap, (long)FAST_BEEP_GAP, (long)SLOW_BEEP_GAP);
  return gap;
}


// ===========================================================================
//  updateBuzzer()  /  עדכון הזמזם
// ---------------------------------------------------------------------------
//  EN: Non-blocking beeper. Uses millis() to toggle the passive buzzer on/off
//      so the beep rate matches the distance. tone()/noTone() drive the
//      passive buzzer through the BC547 transistor without blocking the CPU.
//  HE: זמזם לא-חוסם. משתמש ב-millis() כדי להחליף את מצב הזמזם הפסיבי, כך
//      שקצב הצפצוף תואם למרחק. הפקודות tone()/noTone() מפעילות את הזמזם
//      הפסיבי דרך טרנזיסטור ה-BC547 ללא חסימת המעבד.
// ===========================================================================
void updateBuzzer() {
  long gap = beepGapForDistance();

  // EN: out of range -> make sure the buzzer is silent / HE: מחוץ לטווח - לוודא שקט
  if (gap < 0) {
    if (buzzerIsOn) {
      noTone(BUZZER_PIN);
      buzzerIsOn = false;
    }
    return;
  }

  unsigned long now = millis();

  if (buzzerIsOn) {
    // EN: turn the beep off after BEEP_ON_MS / HE: כיבוי הצפצוף לאחר BEEP_ON_MS
    if (now - lastBeepEdge >= BEEP_ON_MS) {
      noTone(BUZZER_PIN);
      buzzerIsOn   = false;
      lastBeepEdge = now;
    }
  } else {
    // EN: start a new beep after the (distance-based) gap / HE: התחלת צפצוף אחרי המרווח
    if (now - lastBeepEdge >= (unsigned long)gap) {
      tone(BUZZER_PIN, BUZZER_FREQ);
      buzzerIsOn   = true;
      lastBeepEdge = now;
    }
  }
}


// ===========================================================================
//  setup()  /  אתחול ראשוני
// ---------------------------------------------------------------------------
//  EN: Runs once at power-up. Configures pin directions, the echo interrupt,
//      Timer1, and the serial port used for debugging.
//  HE: רץ פעם אחת בהפעלה. מגדיר את כיווני הרגליים, את פסיקת ההד, את Timer1
//      ואת היציאה הטורית המשמשת לניפוי שגיאות.
// ===========================================================================
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);

  // EN: attach the echo interrupt on both edges / HE: חיבור פסיקת ההד לשני המעברים
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  // EN: start the periodic-measurement timer / HE: הפעלת טיימר המדידה המחזורי
  setupTimer1();

  Serial.println(F("Smart Distance Alert System - ready / המערכת מוכנה"));
}


// ===========================================================================
//  loop()  /  הלולאה הראשית
// ---------------------------------------------------------------------------
//  EN: The main loop never blocks. Each pass it: (1) fires a trigger if the
//      timer asked for one, (2) processes any finished echo, (3) refreshes the
//      LEDs and the buzzer, and (4) prints debug info. Everything is driven by
//      flags and millis(), so the CPU is always responsive.
//  HE: הלולאה הראשית אינה חוסמת לעולם. בכל מעבר היא: (1) שולחת טריגר אם
//      הטיימר ביקש זאת, (2) מעבדת הד שהסתיים, (3) מרעננת את הנוריות והזמזם,
//      ו-(4) מדפיסה מידע לניפוי. הכול מונע על ידי דגלים ו-millis(), כך
//      שהמעבד תמיד זמין.
// ===========================================================================
void loop() {
  // (1) EN: a new measurement was scheduled by Timer1 / HE: מדידה חדשה תוזמנה ע"י Timer1
  if (measureDue) {
    measureDue = false;
    startMeasurement();
  }

  // (2) EN: convert a finished echo to a distance / HE: המרת הד שהסתיים למרחק
  updateDistance();

  // (3) EN: drive the outputs / HE: הפעלת הפלטים
  updateLeds();
  updateBuzzer();

  // (4) EN: lightweight debug print, throttled / HE: הדפסת ניפוי קלה, מוגבלת בקצב
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print(F("Distance / מרחק: "));
    Serial.print(currentDistance);
    Serial.println(F(" cm"));
  }
}
