
/* 
    HNF Theseus Schrittmotor Steuerung
    
    Die Steuerung erfolgt normalerweise über SPI;
    eine Übersicht ist im Dokument "Theseus_Aufbau" zu finden.
   
    Durch vier Tasten kann auch eine manuelle Steuerung erfolgen.
    Die Bewegung erfolgt solange, bis entweder die Taste losgelassen,
    die nächste Feldmitte erreicht wird oder ein entsprechender SPI-Befehl
    eintrifft. 

    Für jeden Motor gibt es zwei Endkontakte, die die Bewegung in die 
    entsprechende Richtung beenden, fünf Kontakte für die Feldmitten, 
    und zwei Kontakte für eine Zaun- oder Zielberührung.
    
    Die Position wird in den unteren 3+3 Bits eines Bytes codiert.
    Oktal ist die erste Ziffer die Ost-West- und die zweite die Nord-Süd-Richtung:
        pos = x << 3 | y 
    Null zeigt eine ungültige (Nicht-Mitten-) Position an.
    Entsprechend ist 015 links unten (Süd-West-Ecke)
    und 051 rechts oben (Nord-Ost-Ecke).
    
    Für den Start und Stop einer Bewegung werden Beschleunigungs-  
    und Verzögerungsrampen verwendet, bei denen die Zeit zwischen zwei 
    Schritten (Periode) erhöht bzw. vermindert werden, 
    bis die gewünschte Geschwindigkeit erreicht ist.
    Um beispielsweise von 500Hz (2000µs) auf 5kHz (200µs) zu beschleunigen, 
    wird nach jedem Schritt die Periode um 9µs verkürzt, 
    so dass nach 200 Schritten (2mm) die Maximalgeschwindigkeit erreicht ist.
    Anhalten erfolgt mit 36µs Inkrement, 
    also maximal (2000-200)/36 = 50 Schritten (0.5mm, < 100ms). 
    Anstelle einer zeitlineare Rampe kann beim Beschleunigen eine
    logarithmische Rampe verwendet werden, indem die Zeit nach jedem Schritt
    um denselben Bruchteil der aktuellen Zeit vermindert wird. 
    Ist der Faktor 1/87, sind es auch 200 Schritte, 
    Es wird der Zeitgeber 1 verwendet (schließt die Servo-Bibliothek aus).
    
    Jeder Motor hat drei Anschlüsse:
    - Enable
    - Direction
    - Pulses
    
    Das Enable-Signal benötigt eine kurze Zeit, um wirksam zu werden.
    Der Einfachheit erhalten beide Motoren dieselben Impulse, 
    und es enthält nur einer das ENABLE Signal.
      
    Motor 1 ist der Nord-Süd-Motor, 
    Motor 2 der West-Ost-Motor
    
    Entsprechend dem Original ist die Nord-Süd-Richtung 
    vom Betrachter fort (Nord) und zum Betrachter hin (Süd),
    wenn das Scharnier auf der abgewandten Seite ist.
    Dann ist West-Ost die links-rechts-Richtung.
    
    Bei einer Steigung von 4mm/Umdrehung und 400 Halbschritten/Umdrehung
    ergeben sich 100 Schritte pro mm (1000/cm, 10000 pro Feld, 50000 pro Richtung). 
    Bei der langsamsten Geschwindigkeit von 400/sec sind das 4mm/sec,
    bei der schnellsten von 4000/sec 4cm/sec oder 2.5 sec pro Feld.
    Realistisch sind 2kHz (2cm/sec, 5sec/Feld, 25sec pro Richtung).
 
    Die Schalter der Feldmitten sind nicht per Hardware entprellt.  
    Eine Hardware-Entprellung würde einen Schmitt-Trigger verwenden
    und einen Kondensator mit einem kleinen Widerstand ent- und einem
    wesentlich größeren aufladen, so dass die Verzögerung beim Ansprechen 
    gering ist und auf Seite des Abschaltens liegt.
 
    Die Software-Entprellung könnte über den Weg (die Anzahl der Schritte)
    erfolgen; allerdings wird der Schrittzähler mit jedem Start 
    zurückgesetzt.
    Daher wird eine Zeitschranke von 100ms zur Entprellung verwendet.
 
    SPI
    === 
    
    Anders als mit Hardware-SPI ist die Reaktionszeit des Slaves auf ein 
    eingegangenes Byte unsicher (Interrupt-Reaktion, andere Interrupts,
    System-Software, die Interrupts sperrt).
    
    Ein SPI-Befehl zur Statusabfrage (0x00) oder Positionsabfrage (0xFF)
    wird unmittelbar ausgeführt, indem die Information im Ausgangsregister
    abgelegt und beim nächsten Anruf gesendet wird. 
    Die Antwort auf eine Positions-Abfrage hat das Positions-Bit
    gesetzt, so dass auf eine Positionsabfrage so lange Status-Abfragen
    geschickt werden, bis die Antwort dieses Bit gesetzt hat.
    Entsprechend sollte, wenn die Antwort auf eine Status-Anfrage 
    das Positonsbit gesetzt hat, wiederholt werden.
    
    Ansonsten wird üblicherweise bei einer Status-Anfrage auf das Ready-Bit
    gewartet; sie wird also ohnehin widerholt, so dass ein alter Status
    problemlos ist.
    
    Ein Kommando, das keine Status- oder Positionsabfrage ist,
    wird in einem Puffer für die Hauptschleife bereitstellt und
    zugleich das Ready-Bit gelöscht und dieser Status als Antwort
    bereitgestellt. 
    Wurde das Kommando im Puffer noch nicht bearbeitet, so wird das
    empfangene Byte verworfen und das Overrun-Bit im Status gesetzt und
    als Antwort bereitgestellt.
    Das Ready-Bit wird dann nicht verändert und sollte gelöscht sein.
    Sobald ein Kommando (weder Status noch Position) empfangen wird und
    der Puffer frei ist, wird das Overrun-Bit (wie auch das Ready-Bit)
    gelöscht. 
    Der Master sollte also nach jedem Befehl den Status holen:
    Wenn das Positons-Bit gesetzt ist, neu holen.
    Wenn das Overrun-Bit gesetzt ist, neu senden.
    Damit das Ready-Bit auf jeden Fall gelöscht ist,
    wird bei einem Kommando ohne Bewegung eine Mindestdauer gewartet.
    
    wird das 
    wird als bereitgestellt markiert und verworfen, wenn es noch nicht
    bearbeitet ist; dann wird das MOTOR_StatusOverrunBit gesetzt.
    Es wird wieder gelöscht, wenn das Puffer-Byte wieder frei war;
    dann wird auch bereits in der Interrupt-Routine das MOTOR_StatusReadyBit
    gelöscht; unabhängig davon, ob eine Bewegung gestartet wird.
    Es wird gesetzt, wenn die Motoren im Stillstand sind.
    
    Das Bit WCOL im SPI-Statusregister wird nicht abgefragt.
    Dieses zeigt an, dass das Datenregister nicht verändert wurde;
    es wird also ein veraltetet Status übertragen. Dieser Fall wird
    von den obigen Abläufen mit umfasst. 
    
    Um Zeitproblemen mit den Diagnoseausgaben über die serielle
    Schnittstelle zu entgehen, werden diese nur ausgegeben,
    nachdem ein (beliebiges) Zeichen empfangen wurde.
 */
 
const char version [] = "2.1b";

bool debug = false;

#define PulsesPerField 10000
#define minimalCommandTime 100      // ms

// SPI Bibliothek
#include "SPI.h"

// Kommando- und Status-Codes
#include "SPI_Motor.h"


// Pin Konfigurationen
#define poti1       A0
#define poti2       A1
#define touchFence  3
#define touchGoal   2

// Motor 1 (Nord-Süd)
#define M1_pulse    5          
#define M1_dir      7
#define M1_enable   4
#define M1_stopN    48
#define M1_stopS    36              

#define M1_btn1     A2
#define M1_btn2     A3

#define M1_field1   24
#define M1_field2   26
#define M1_field3   28
#define M1_field4   30
#define M1_field5   32



// Motor 2 (Ost-West)
#define M2_pulse    6
#define M2_dir      8
#define M2_enable   9
#define M2_stopE    22
#define M2_stopW    34

#define M2_btn1     A4
#define M2_btn2     A5

#define M2_field1   38
#define M2_field2   40
#define M2_field3   42
#define M2_field4   44
#define M2_field5   46



/*
    Tabellen für Pin-Zuordungen
*/
enum MotPins { MotPulse, MotDir, MotEnable, MotStop1, MotStop2, MotBtn1, MotBtn2, MotPoti };
byte m1Pins[] = {M1_pulse, M1_dir, M1_enable, M1_stopS, M1_stopN, M1_btn1, M1_btn2, poti1 };
byte m2Pins[] = {M2_pulse, M2_dir, M2_enable, M2_stopW, M2_stopE, M2_btn1, M2_btn2, poti2 };
    
/**
    Interrupt-Routinen
**/

/*
 * Pulsgeber
 * Da der minimale Schrittimpulsabstand mit 200µs (5kHz) recht kurz sein kann,
 * wird der Interrupt von Zeitgeber 1 verwendet
 *
 * Zudem werden keine Schalter abgefragt, 
 * da auch noch Zeit für den SPI-Interrrupt benötigt wird.
 */

/*
 * Interrupt-Routine
 * 
 * Das Verhalten wird durch 'timerState' bestimmt:
 *  - 'timerOff':   es werden keine Impulse erzeugt 
 *                   und keine anderen Variablen verwendet.
 *  - 'timerStart': Pulsdauer in 'actualPeriod' bis 'minPeriod' vermindern,
 *                   dann auf 'timerOn' wechseln.
 *                   Die Verminderung erfolgt wahlweise linear oder exponentiell.
 *  - 'timerStop':  Pulsdauer in 'actualPeriod' um 'rampIncr' bis 'maxPeriod' erhöhen,
 *                   dann auf 'timerOff' wechseln.
 *  - 'timerOn':    Pulse werden ohne Veränderung ausgegeben, 
 *                   nur 'pulseCount' wird vermindert und geprüft.
 *
 * Die Übergänge von 'timerOn', 'timerStop' und 'timerOff'  erfolgen
 * hier automatisch.
 * 
 * Zum Starten wird 'timerState = timerStart' verwendetn,
 * zum Anhalten 'timerState = timerStop'.
 * Eine vorherige Prüfung auf 'timerOff' ist weder notwendig
 * noch sinnvoll (race condition).
 *
 * Als zuverlässige Anzeige, dass die Motoren angehalten sind,
 * dient die bool'sche Variable 'pulsesActive', die hier
 * gesetzt und gelöscht wird.
 * 
 * Mit jedem Impuls wird die Variable 'pulseCount' vermindert;
 * wenn sie Null ist, wird die Bewegung per 'timerStop' angehalten.
 *
 * Vor dem Setzen des 'timerState' auf 'timerStart' ist 'pulsesCount'
 * auf die gewünschte maximale Fahrlänge (ohne Bremsweg) zu setzen;
 */
#define periodMax 1333                // mind.750 Hz
#define periodMin 300                 // max. 3333 Hz
#define rampIncr 36                   // Increment der Stoprampe (µs)
#define rampDecr 18                   // Decrement der Startrampe (µs)
#define rampFactor 87                 // Faktor exponentieller Anstieg
#define RAMP_EXP 0                    // Exponentielle statt linearer Rampe   
// Flags, um die Impulse zu steuern
enum timerStates { timerOff, timerStart, timerStop, timerOn};
volatile byte timerState = timerOff; // Steuerung
volatile bool pulsesActive = false;
volatile unsigned pulseCount = 0;     // Pulszähler 
unsigned minPeriod = 200;             // µs; Endwert fürs Beschleunigen
unsigned maxPeriod = 4000;            // µs; Endwert fürs Bremsen
unsigned actualPeriod = 2000;         // µs; Pulsdauer aktuell


#define TICKS 2 // F_CPU/1000000/8       // Prescaler dividiert durch 8, also 2/µs bei 16MHz
/* 
 *  Interrupt-Handler für Schrittmotorsteuerung
 */
ISR(TIMER1_COMPA_vect) { 

    // wenn angehalten, nichts tun
    if (timerState == timerOff) {
        return;
    }
    
    // anhalten, wenn maximale Distanz erreicht
    if (pulseCount == 0) {                
        timerState = timerStop;
        pulseCount =  PulsesPerField;       // Überlauf ist unschön
    } 

    // Beschleunigung ?
    if (timerState == timerStart) {
#if RAMP_EXP
        actualPeriod -= actualPeriod/100; 
#else
        actualPeriod -= rampDecr;  
#endif
        if (actualPeriod <= minPeriod) {
            timerState = timerOn;
            actualPeriod = minPeriod;
        }
        OCR1A = TICKS * actualPeriod;   
    }
        
    // Verzögerung ?
    if (timerState == timerStop) {
        actualPeriod += rampIncr;
        if (actualPeriod >= maxPeriod) {
            timerState = timerOff;
            actualPeriod = maxPeriod;
            // Motoren ausschalten für weniger abruptes Anhalten
            digitalWrite(M1_enable, HIGH);
            digitalWrite(M2_enable, HIGH);
            pulsesActive = false;           
            return;                         // keine weiteren Impulse
        }
        OCR1A = TICKS * actualPeriod;   
    }
    
    pulsesActive = true;                 
    digitalWrite(M1_pulse, LOW);
    digitalWrite(M2_pulse, LOW);
    --pulseCount;                           // reicht für Pulsbreite aus
    digitalWrite(M1_pulse, HIGH);
    digitalWrite(M2_pulse, HIGH);
    
}

/*
 * SPI interrupt routine
 *
 * Called whenever a SPI exchange has been done
 * The value sent was in the data register,
 * the value received is in the same,
 * which must be replaced by the value to be sent on the next request,
 * that is normally the status byte,
 * unless the command received is 0xFF, then it is the positon byte.
 * 
 * The interrupt sets a flag that a new command (neither status nor position)
 * has arrived; the main loop resets it if the buffer is no longer used
 * and can be filled with the next command.
 * If it is still set when a new command (neither status nor position)
 * arrived, the command is discarded and the Overrun Flag set.
 * It is reset when another command is accepted.
 * 
 * Write collisions (WCOL bit in the status) are not queried.
 * While the byte thus sent is outdated, another request
 * will supply a more recent value.
 */
// Globla variables for communication with the main loop:
volatile bool spiNewCommand = false;
volatile byte spiStatus, spiPosition;   // the value(s) to be sent next
volatile byte spiCommand;               // the command received

// SPI interrupt routine
ISR (SPI_STC_vect) {

    byte newval = SPDR;                 // angekommenes Byte
    
    // Positionsanfrage?
    if (newval == 0xFF) {       
        SPDR = spiPosition | bit(MOTOR_StatusPositionBit);  
        return;                 
    }
    // Statusanfrage?
    if (newval == 0) {          
        SPDR = spiStatus;           // Position-Bit ist immer aus
        return;
    }
    
    // Kommando noch nicht akzeptiert?
    if (spiNewCommand) 
        bitSet(spiStatus, MOTOR_StatusOverrunBit);
    else {
        bitClear(spiStatus, MOTOR_StatusReadyBit);
        bitClear(spiStatus, MOTOR_StatusOverrunBit);
        spiCommand = newval; 
        spiNewCommand = true;       // Kommando angekommen
    }
    SPDR = spiStatus;               
}  


/*
 * Pulsgeber initialisieren 
 */
void initPulses() {
    // Keine Impulse ausgeben
    timerState = timerOff;
    pulsesActive = false;

    // Hardware initialisieren
    TIMSK1 = 0;                         // stop timer, no need to disable interrupts globally
    TCCR1A = 0;                         // normal operation
    TCCR1B = bit(WGM12) | bit(CS11);    // CTC, pre-scaling 8
    OCR1A =  maxPeriod;                 // compare A register value
    TIMSK1 = bit (OCIE1A);              // start hardwar timer in "interrupt on Compare A Match" mode
}



 /*
    Tastaturabfrage
    Es wird ein Bitmuster für die gedrückten Tasten zurückgegeben;
    eine Tastenänderung muss vom Aufrufer erkannt werden.
    Auch wenn eine Hardware-Entprellung vorhanden ist,
    wird das neue Bitmuster nur zurückgegeben, 
    wenn es lange genug stabil ist.
 */
// Tastencodes; 0 = kein Taste
#define K1 1
#define K2 2
#define K3 4
#define K4 8
const int debounceTime = 500;
int getKey() {
    int actkey = 0;
    if (digitalRead(M1_btn1) == LOW) actkey |= K1;
    if (digitalRead(M1_btn2) == LOW) actkey |= K2;
    if (digitalRead(M2_btn1) == LOW) actkey |= K3;
    if (digitalRead(M2_btn2) == LOW) actkey |= K4;
    
    static int lastkey = 0;         // letztes gültiges Muster
    if (actkey == lastkey)          // keine Änderung
        return lastkey;
 
    // Entprellung
    unsigned long timeLastChanged = 0;
    static int prevkey = 0;         // vorheriges Muster während Wartezeit
    if (prevkey != actkey) {
        // Tastenänderung, Zeit merken
        timeLastChanged = millis();
        prevkey = actkey;
        return lastkey;             // noch keine Tastenänderung
    }

    // neues Muster ist stabil; aktiviere, wenn Wartezeit abgelaufen
    if (millis() - timeLastChanged > debounceTime) 
        lastkey = actkey;
    
    return lastkey;
}


/*
    Potentiometer auslesen für Start- und Höchstgeschwindigkeit
*/
void getSpeed() {
    // Anlauf-Geschwindigkeit, maximale Periodendauer
    int newmax = map(analogRead(poti1), 0, 675 /* 3.3 statt 5V: 1023 */, periodMax, periodMin);
    // Fahr-Geschwindigkeit, minimale Periodendauer
    int newmin = map(analogRead(poti2), 0, 675 /* 3.3 statt 5V: 1023 */, periodMax, periodMin);
    static int prevmin = 0, prevmax = 0;
    if (abs(newmin - prevmin) > 20 || abs(newmax - prevmax) > 20) {       
        prevmin = newmin;
        prevmax = newmax;
        // change only if not moving
        if (!pulsesActive) {
            maxPeriod = newmax;
            minPeriod = min(newmin, newmax);   
            actualPeriod = maxPeriod;           // beim nächsten Start
            if (debug) {
                Serial.print("Start Period: ");
                Serial.print(maxPeriod);
                Serial.print(" Run Period: ");
                Serial.println(minPeriod);
            }
        }
    }
} 

/*
   Motor starten.
   Der Motor soll im Stillstand sein.
   'pulseCount' muss vom Aufrufer zuvor gesetzt werden.
 */

void runMotor(byte mot[], int mdir) {

    // Fehler, wenn nicht Stillstand
    unsigned long now = millis();
    bool runErr = false;;
    while (pulsesActive) {
        if (runErr == false)
            Serial.print("Waiting for stop: ");
        runErr = true;
    }
    if (runErr) {
        Serial.print(millis() - now);
        Serial.println("ms");
    }
    
    
    // beide Motorstränge abschalten
    digitalWrite(M1_enable, HIGH);
    digitalWrite(M2_enable, HIGH);
    
    // Nicht starten, wenn Grenzschalter in der gegebenen Richtung aktiv
    if (digitalRead(mot[MotStop1]) == LOW && mdir == LOW)
        return;
    if (digitalRead(mot[MotStop2]) == LOW && mdir == HIGH)
        return;
    
    // gewünschten Motor anschalten und Richtung setzen
    digitalWrite(mot[MotEnable], LOW);
    delay(10);
    digitalWrite(mot[MotDir], mdir);
    delay(10);
    // starten
    timerState = timerStart;
    // Warte auf Start der Motoren, um vorzeitiges Ready zu vermeiden
    while (!pulsesActive)
        ;
    
    if (debug) {
        Serial.print("runMotor ");
        if (mot == m1Pins)
            Serial.print(1);
        else 
            Serial.print(2);
        Serial.print("->");
        Serial.print(mdir);
        Serial.print(" cnt=");
        Serial.println(pulseCount);
    }

     
    
}


/** 
    Globale Variable zur Steuerung der Hauptschleife
**/
byte loopStopFieldCount = 0;        // Maximalzahl der Felder
bool stopOnGoalOrFence = true;        // Ziel- und Zaunkontakte auswerten



/*
    Bewegung starten in Richtung 'dir' um 'fields' Felder.
    'fields == 0' bedeutet maximal ein halbes Feld (Feldmitte ab Zaun)
    runMotor() startet nicht, wenn der entsprechende Grenzschalter aktiv ist
*/
byte lastMotorDir = MOTOR_DirInvalid;      // letzte verwendete Richtung

void doMotion(byte dir, byte fields) {
 
    // Richtung der als letztes eingeleiteten Bewegung merken
    lastMotorDir = dir;
    
    // Begrenzung der Bewegung
    if (fields < 1)
        pulseCount = PulsesPerField / 2;
    else
        pulseCount = fields * PulsesPerField;   
    pulseCount += PulsesPerField / 10;             // grace distance
    switch (dir) {
        case MOTOR_DirSouth:    
            runMotor(m1Pins, LOW);
            break;
        case MOTOR_DirNorth:    
            runMotor(m1Pins, HIGH);
            break;
        case MOTOR_DirWest:    
            runMotor(m2Pins, LOW);
            break;
        case MOTOR_DirEast:    
            runMotor(m2Pins, HIGH);
            break;
    } 
    

}
 
 /*
   Abfrage der Endschalter.
   Wenn ein Endschalter aktiv ist, 
   wird das Limit-Bit im Status gesetzt
   und ein Stop eingeleitet.
   
   Entprellen ist nicht notwendig, weil nach dem Ende der Bewegung
   der Schalter permanent eingeschaltet sein sollte.
*/
#if 0
void checkLimitSwitches() {
    // Wenn kein Endschalter aktiv ist, fertig
    if (digitalRead(M1_stopS) != LOW && digitalRead(M1_stopN) != LOW &&
        digitalRead(M2_stopW) != LOW && digitalRead(M2_stopE) != LOW ) {
        bitClear(spiStatus, MOTOR_StatusLimitBit);
        return;   
    }
    
    // mindestens ein Endschalter ist aktiv
    bitSet(spiStatus, MOTOR_StatusLimitBit); 
        
    // Anhalten, denn es ist die falsche Richtung 
    timerState = timerStop;
}
#else
void checkLimitSwitches() {
    // Endschalter -> Status (live)
    if (digitalRead(M1_stopN) == LOW || digitalRead(M1_stopS) == LOW ||
        digitalRead(M2_stopE) == LOW || digitalRead(M2_stopW) == LOW )
        bitSet(spiStatus, MOTOR_StatusLimitBit); 
    else
        bitClear(spiStatus, MOTOR_StatusLimitBit); 
        
    // keine weiter Aktion, wenn keine aktive Bewegung
    if (!pulsesActive)
        return;
    
    if (digitalRead(M1_enable) == LOW) {
      if (digitalRead(M1_stopS) == LOW && digitalRead(M1_dir) == LOW) 
          timerState = timerStop;
      if (digitalRead(M1_stopN) == LOW && digitalRead(M1_dir) == HIGH) 
          timerState = timerStop;
    }
        
    if (digitalRead(M2_enable) == LOW) {
      if (digitalRead(M2_stopW) == LOW && digitalRead(M2_dir) == LOW)
          timerState = timerStop;
      if (digitalRead(M2_stopE) == LOW && digitalRead(M2_dir) == HIGH)
          timerState = timerStop;
    }
}
#endif
/*
 * Abfrage der Zaun- und Zielkontakte;
 * Ergebnis ist in den Status-Bits.
 * 
 * Für beide Kontakt kann eine gemeinsame Entprellung
 * verwendet werden, da nie beide gleichzeitig aktiv sind.
 *
 */
#define debounceFGTime 100    // milliseconds
void checkFenceAndGoalSwitches() {

    static long unsigned debounceStart = 0;
    
    // Zaunkontakt?
    if (digitalRead(touchFence) == HIGH) {            
        bitSet(spiStatus, MOTOR_StatusFenceBit);
        debounceStart = millis();
        return;  
    }

    // Zielkontakt?
    if (digitalRead(touchGoal) == HIGH) {
        bitSet(spiStatus, MOTOR_StatusGoalBit);
        debounceStart = millis();
        return;
    }

    // Entprellung
    if ((millis() - debounceStart) < debounceFGTime)
        return;
    
    // konsolidierter Status
    bitClear(spiStatus, MOTOR_StatusFenceBit);
    bitClear(spiStatus, MOTOR_StatusGoalBit);

}


/*
    Abfrage der Feldmittenschalter:
    
    Wenn einer der Schalter geändert ist, wird die entsprechende
    Position in 'spiPosition' hinterlegt.
    
    Eine Entprellzeit verhindert, dass ein Öffnen währenddessen
    eine Statusänderung bewirkt.
    
    Wenn die Position in beiden Achsen gültig ist,
    wird das MID-Bit in 'spiStatus' gesetzt, sonst gelöscht.
   
 */
int m1_ports[5] = { M1_field1, M1_field2, M1_field3, M1_field4, M1_field5 };
int m2_ports[5] = { M2_field1, M2_field2, M2_field3, M2_field4, M2_field5 };

#define MidDebounce 100   // msec

void checkMidSwitches() {
    byte i;
    byte midNS=0, midEW=0;
    static byte oldMidNS = -1, oldMidEW = -1;
    static long unsigned debounce = 0;        // Zeitpunkt per millis()
            
    // Abfrage der Schalter
    for (i=0; i<5; ++i) 
        if (digitalRead(m1_ports[i]) == LOW) {
            midNS = 5 - i;
            break;          // andere können nicht gesetzt sein
        }
    for (i=0; i<5; ++i) 
        if (digitalRead(m2_ports[i]) == LOW) {
            midEW = i+1;
            break;          // andere können nicht gesetzt sein
        }
        
    // während der Entprellzeit werden Öffnungen ignoriert
    if (midNS == 0 || midEW == 0)
        if (millis() - debounce < MidDebounce)
            return;

    // keine Änderung: fertig
    if (midNS == oldMidNS && midEW == oldMidEW)
        return;
    // Entprellung auch in der Position
    if (millis() - debounce < MidDebounce)
      return;

    
    oldMidNS = midNS;
    oldMidEW = midEW; 
      
    // Nach Änderung Entprellzeit aktivieren
    debounce = millis();

    // Position atomar setzen
    volatile byte newpos = (midEW<<3) | midNS;      
    spiPosition = newpos;          
        
    // Statusbit bestimmen: Feldmitte in beiden Achsen 
    if (midNS > 0 && midEW > 0) 
        bitSet(spiStatus, MOTOR_StatusMidBit);
    else
        bitClear(spiStatus, MOTOR_StatusMidBit);
}
 
/* 
    Feldwechsel erkennen und dann Feldzähler vermindern;
    wenn aufgebraucht, Motor anhalten.
    Muss unmittelbar nach checkMidSwitches() aufgerufen werden.
    werden, damit die Position nicht wieder verschwindet
*/
void checkFieldChangeCounter() {
    static byte oldpos = 0;
    
    // Nur prüfen, wenn in der Mitte
    if (bitIsClear(spiStatus, MOTOR_StatusMidBit)) 
        return;

    // Wenn Feldwechsel, Zähler vermindern
    if (oldpos != spiPosition) {
        oldpos = spiPosition;
        if (loopStopFieldCount > 0)     
            loopStopFieldCount -= 1;
    }
    
    // Ggf. anhalten
    if (loopStopFieldCount < 1) {
      timerState = timerStop;
      loopStopFieldCount = 0;
    }
}
 
/*
    Motore auf Feldmitte normieren (ohne Ziel- und Zaun-Auswertung)
    
    Weil mehrere Bewegungen notwendig sein können, 
    wird nach dem Starten einer Bewegung zurückgekehrt.
    Daher muss solange aufgerufen werden, bis das Mid-Bit im Status gesetzt ist; 
    dann sind beide Koordinaten in 'spiPosition' gültig (nicht 0)
    Dann kann noch die letzte Bewegung in der Bremsphase sein;
    daher muss ggf. auf das Ende der Bewegung gewartet werden.
            
    Wenn ein Endschalter aktiv ist, wird zunächst eine Bewegung 
    in Gegenrichtung gestartet.
    
    Ansonsten wird bei einer ungültigen Positonskoordinate
    nach Ost bzw. Süd bewegt; dabei kann ein Endschalter ansprechen 
    und die Bewegung umgekehren. 
    
    Da diese Funktion in der Regel keine Veränderung bewirkt, weil
    die Maschine normalerweise in Feldmittenposition stehen bleibt,
    ist eine Optimierung der Richtung bei ungültiger Mittenposition
    nicht sinnvol.
*/
static const byte toCenterLimitMoveOff = 0;
static const byte toCenterLimitMoveNS = 1;
static const byte toCenterLimitMoveEW = 2;

void toCenter() {
    static byte toCenterLimitMove = toCenterLimitMoveOff;
    
    // Endschalter immer sofort auswerten
    byte leaveDir = MOTOR_DirInvalid;
    
    if (digitalRead(M1_stopS) == LOW) 
        leaveDir = MOTOR_DirNorth;
    if (digitalRead(M1_stopN) == LOW) 
        leaveDir = MOTOR_DirSouth;
    if (digitalRead(M2_stopW) == LOW) 
        leaveDir = MOTOR_DirEast;
    if (digitalRead(M2_stopE) == LOW) 
        leaveDir = MOTOR_DirWest;
   
    if (leaveDir != MOTOR_DirInvalid) {
        // Endschalter (noch) aktiv, anhalten wenn falsche Richtung
        if (leaveDir != lastMotorDir) {
            // Sofort anhalten und darauf warten
            timerState = timerStop;     // ohne Wirkung wenn nicht PulsesActive
            while (pulsesActive)
                    delay(1);           // delay nicht notwendig; Platzhalter
        } 
        
        // wenn hier die Bewegung aktiv ist, dann in richtiger Richtung
        if (pulsesActive)
            return;                     // abwarten und Mittenschalte auswerten
        
        // Gegenbewegung muss gestartet werden
        doMotion(leaveDir, 1);          // kann mehr als ein halbes Feld sein   
        if (leaveDir == MOTOR_DirNorth || leaveDir == MOTOR_DirSouth)
            toCenterLimitMove = toCenterLimitMoveNS;
        else
            toCenterLimitMove = toCenterLimitMoveEW;
        if (debug) {
            Serial.print("center limit move: ");
            Serial.println(leaveDir);
        }
        return;
    }

    
    // Mittenschalter abfragen; wenn Mitte erreicht, fertig
    checkMidSwitches();         // aktualisiert spiPosition und spiStatus 
    
    // wenn das Mid-Bit erkannt ist, anhalten und fertig
    // Redundanter Code, nur der Klarheit wegen
    if (bitIsSet(spiStatus, MOTOR_StatusMidBit)) {
        timerState = timerStop;
        return;
    }
    
    // Ost-West Position auswerten
    if (MOTOR_PosX(spiPosition) == 0)
        leaveDir = MOTOR_DirEast;           // merken      
    else {
        // gültige Ost-West Position, LimitMove ggf. anhalten
        if (toCenterLimitMove == toCenterLimitMoveEW) {
            timerState = timerStop;         
            toCenterLimitMove = toCenterLimitMoveOff;
            return;
        }
    }
        
    // Nord-Süd  Position auswerten
    if (MOTOR_PosY(spiPosition) == 0) 
        leaveDir = MOTOR_DirNorth;          // merken 
    else {
        // gültige Nord-Süd Position, LimitMove ggf. anhalten
        if (toCenterLimitMove == toCenterLimitMoveNS) {
            // gültiger Mittenschalter
            timerState = timerStop;         // anhalten
            toCenterLimitMove = toCenterLimitMoveOff;
            return;
        }
    }
    
    // wenn noch wg. Grenzschaltern unterwegs, laufen lassen
    if (toCenterLimitMove != toCenterLimitMoveOff)
        return;
    
    // Wenn Merker ungültig, sind beide Postionen sind ok, also Mittenposition
    if (leaveDir == MOTOR_DirInvalid) {
        // Mittenposition erreicht, anhalten
        timerState = timerStop;     
        // das Mid-Bit ist bereits gesetzt
        return; 
    }
   
    // Wenn falsche Richtung, anhalten
    if (leaveDir != lastMotorDir) {
        timerState = timerStop;   
        // sofort auf Erledigung warten
        while (pulsesActive)
            delay(1);                 // delay nicht notwendig, Platzhalter
    }
    
    // wenn noch nicht in Bewegung, gemäß 'leaveDir' starten 
    if (!pulsesActive) {
        doMotion(leaveDir, 0);        // maximal ein halbes Feld
        if (debug) {
            Serial.print("To mid: ");
            Serial.println(leaveDir);
        }
    }
}  
 
/**
    SPI Kommando ausführen
**/
void doSPIcommand(byte cmd) {

    if (debug) {
        Serial.print("SPI: ");
        Serial.print(spiCommand, HEX); 
        Serial.print(" status: ");
        Serial.print(spiStatus, BIN);  
        if (bitIsSet(spiStatus, MOTOR_StatusReadyBit))  
          Serial.print(" ready");
        else 
          Serial.print(" not ready");  
        if (pulsesActive)
          Serial.print(" active");
        else
          Serial.print(" not active");
        Serial.println();        
    }
 
    // Status- und Positions-Abfrage sind schon erledigt
    
    // Kommando-Byte zerlegen
    byte op = MOTOR_CmdCommand(cmd);
    byte dir = MOTOR_CmdDirection(cmd);
    byte count = MOTOR_CmdCount(cmd);
 
    // Anhalten
    if ( op == MOTOR_CmdStop) {
        timerState = timerStop;    
        return;
    }

    // Zug-Bewegung mit Ziel- oder Zaun-Halt
    if ( op == MOTOR_CmdDrag) {
        stopOnGoalOrFence = true;
        loopStopFieldCount = count;
        doMotion(dir, count);
        return;
    }
    
    // Fahr-Bewegung ohne Ziel- oder Zaun-Halt
    if ( op == MOTOR_CmdMove) {
        stopOnGoalOrFence = false;
        loopStopFieldCount = count;
        doMotion(dir, count);
        return;
    }
    
    // Auf Feldmitte (ohne Ziel- oder Zaun-Halt)
    // Befehle incl. Stop können verloren gehen
    if ( op == MOTOR_CmdCenter) {
        while (bitIsClear(spiStatus, MOTOR_StatusMidBit))
           toCenter();   
        // Sender muss noch auf das Ready-Bit warten
        return;
    }
    
   
    // Unbekanntes Kommando

    // Da keine Bewegung angestoßen wird, setzt 'loop' das Ready bit
    if (debug) 
        Serial.println(" -- ignored");
}





/**
    Hauptschleife 
    Hier werden die Bewegungen initiert 
    und überwacht.
**/
void loop() {
    
    // neues SPI Kommando?
    if (spiNewCommand) {
        byte cmd = spiCommand;
        // Der Interrupt muss das Ready-Bit gelöscht haben
        if (bitIsSet(spiStatus, MOTOR_StatusReadyBit)) {
            if (debug) {
              Serial.print("*** SPI Command  ");
              Serial.print(cmd, HEX);
              Serial.print(" ready, status: ");
              Serial.println(spiStatus, BIN);
            }
            bitClear(spiStatus, MOTOR_StatusReadyBit); 
        }
        spiNewCommand = false;              // Puffer frei  
        doSPIcommand(cmd);   
        if (debug) {
          Serial.print("Status after command do:");
          Serial.println(spiStatus, BIN);  
        }
        return;
    }
    
    // Grenzschalter auswerten; leitet ggf. sofort einen Stop ein
    checkLimitSwitches();
        
    // Ziel- und Zaun auswerten und ggf. anhalten
    checkFenceAndGoalSwitches();   
    if (stopOnGoalOrFence)                    // wird von doSPIcmd() gesetzt
        if (bitIsSet(spiStatus, MOTOR_StatusGoalBit)
         || bitIsSet(spiStatus, MOTOR_StatusFenceBit))
            timerState = timerStop;

    // Feldmitten auswerten
    checkMidSwitches();                     // setzt MID-Bit und Position
    checkFieldChangeCounter();              // Feldwechsel behandeln
    
    // Motor-Stillstand ist Befehlsende 
    
    /* Grundsätzlich sollte ein Kommando abgeschlossen sein
       sobald die Motoren im Stillstand sind.
       Durch einen noch nicht geklärten Programmierfehler
       wurde das Ready-Bit vorzeitig gesezt;
       die folgende Verzögerung ist daher eine provisorische
       Maßnahme.
    */
    if (!pulsesActive && !spiNewCommand) {
        delay(10);
        // still inactive?
        if (!pulsesActive && !spiNewCommand)
            bitSet(spiStatus, MOTOR_StatusReadyBit);
        else
          if (debug)
            Serial.println("Premature Motor inactive");
    } 

    // nur geänderten Status ausgeben 
    // evtl. Serial.availableForWrite() verwenden
    static byte oldstatus = 0, oldposition =0, oldcount = 0;
    if (debug) {
        if (spiStatus != oldstatus) {
            oldstatus = spiStatus;
            Serial.print("new status= ");
            Serial.println(oldstatus, BIN);       // no need to use volatile
        } 
        
        if (loopStopFieldCount != oldcount) {
            oldcount = loopStopFieldCount;
            Serial.print("new field count=");
            Serial.println(oldcount);
        }
        
        if (spiPosition != oldposition) {
            oldposition = spiPosition;
            Serial.print("new position=");
            Serial.println(oldposition, OCT);
        }
    }

    // Geschwindigkeitsvorgaben (Potis) aktualisieren
    // getSpeed();
    
    // Debug?
    if (Serial.available() > 0) {
        debug = true;
        while (Serial.available() > 0)
            Serial.read();
    }
        
    
    // Tasten befolgen
    static int oldkey = 0;
    int newkey = getKey();
    // Tastenänderung ?
    if (oldkey == newkey) 
        return;                 // keine Taste geändert      
        
    // Tastenänderung
    if (debug) {
        Serial.print("Key=");
        Serial.println(newkey);
    }
    oldkey = newkey;
    loopStopFieldCount = 0;     // nur bis zur nächsten Feldmitte
    switch (newkey) {
        case K1:    
            bitClear(spiStatus, MOTOR_StatusReadyBit);
            doSPIcommand(MOTOR_SPIcommand(MOTOR_DirSouth, 1, MOTOR_CmdDrag));
            break;
        case K2:    
            bitClear(spiStatus, MOTOR_StatusReadyBit);
            doSPIcommand(MOTOR_SPIcommand(MOTOR_DirNorth, 1, MOTOR_CmdDrag));
            break;
        case K3:    
            bitClear(spiStatus, MOTOR_StatusReadyBit);
            doSPIcommand(MOTOR_SPIcommand(MOTOR_DirWest, 1, MOTOR_CmdDrag));
            break;
        case K4:    
            bitClear(spiStatus, MOTOR_StatusReadyBit);
            doSPIcommand(MOTOR_SPIcommand(MOTOR_DirEast, 1, MOTOR_CmdDrag));
            break;
        // keine Taste oder Tastenkombination: Anhalten
        default:
            doSPIcommand(MOTOR_SPIcommand(0, 0, MOTOR_CmdStop));     
            break;
    } 
    
    

 }


/**
  Intialisierung
**/

void setup() {
    /* Serielle Kommunikation
     Geschwindkeit muss hoch sein, sonst könnten Signale verpasst werden
     TODO: Input-Pins mit Flankensteuerung auswerten
    */
    Serial.begin(115200); 
    Serial.println();
    Serial.print("\nMotor Control ");
    Serial.print(version);
    Serial.print(" compiled ");
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);

    /* Pin Modes
    */
    // gemeinsam
    pinMode(poti1, INPUT);
    pinMode(poti2, INPUT);  
    pinMode(touchFence, INPUT);  
    pinMode(touchGoal, INPUT); 
    // Motor 1
    pinMode(M1_pulse, OUTPUT);
    pinMode(M1_dir, OUTPUT);
    pinMode(M1_enable, OUTPUT);
    pinMode(M1_stopS, INPUT);       
    pinMode(M1_stopN, INPUT);
    pinMode(M1_btn1, INPUT);
    pinMode(M1_btn2, INPUT);
    pinMode(M1_field1, INPUT);
    pinMode(M1_field2, INPUT);
    pinMode(M1_field3, INPUT);
    pinMode(M1_field4, INPUT);
    pinMode(M1_field5, INPUT);

    // Motor 2
    pinMode(M2_pulse, OUTPUT);
    pinMode(M2_dir, OUTPUT);
    pinMode(M2_enable, OUTPUT);
    pinMode(M2_stopW, INPUT);
    pinMode(M2_stopE, INPUT);
    pinMode(M2_btn1, INPUT);
    pinMode(M2_btn2, INPUT);
    pinMode(M2_field1, INPUT);
    pinMode(M2_field2, INPUT);
    pinMode(M2_field3, INPUT);
    pinMode(M2_field4, INPUT);
    pinMode(M2_field5, INPUT);

    /* SPI aktivieren
    */
    // SPI Pin modes:
    pinMode(MOSI, INPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SS, INPUT);

    // Enable SPI in slave mode and enable interrupts
    bitClear(SPCR, MSTR); 
    bitSet(SPCR, SPE);
    SPI.attachInterrupt();

    /* Motor-Ansteuerung aktivieren
    */  
    // vor dem Einrichten des Impulsgebers Motoren abschalten
    digitalWrite(M1_enable, HIGH);
    digitalWrite(M2_enable, HIGH);
    // Pulgeber aufsetzen
    initPulses();

    debug = true; 
    // Geschwindigkeiten holen
    getSpeed();

    /* Feldmitte sicherstellen 
    */
    Serial.print("Centering start at ");
    Serial.println(spiPosition, OCT);
    do {
        toCenter();             // setzt ggf. das Mid-Bit
        // durch Taste abbrechen
        if (getKey() != 0)
            break;
    } while (bitIsClear(spiStatus, MOTOR_StatusMidBit));
    // Auf Bewegungsende warten
    while (pulsesActive)
        delay(1);                   // nicht notwendig, Platzhalter
    Serial.print("Centering done at ");
    Serial.println(spiPosition, OCT);
    debug = false;
    if (!debug)
    Serial.println("Send any character to start debug mode.");
}
