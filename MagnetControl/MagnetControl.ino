/* 
 *  Theseus Magnet-Steuerung
 *  
 *  Normalerweise erfolgt die Steuerung über SPI mit einem Byte,
 *  wobei Bit 0..1 die Richtung angibt und Bit 2 einschaltet:
 *     4: Nord  5: West  6: Süd  7: Ost
 *  Alle anderen Werte schalten die Magnete aus.
 *  Es wird immer das zuvor gesendete Byte geantwortet.
 *  
 *  Weitere Details siehe Theseus_Aufbau.
 *  
 *  Getestet werden kann mit den Tasten wie folgt:
 *  Vier Taster auf den Pins A1 bis A4 bestromen die Spulen,
 *  wobei die Richtungen entgegen dem Uhrzeigersinn gewählt sind
 *  (und nicht paarweise für die Spulen).
 *      A1: Nord
 *      A2: West
 *      A3: Süd
 *      A4: Ost
 *  Mit dem Poti an A0 wird das Tastverhältnis, d.h. 
 *  die an der jeweiligen Spule wirksame Magnetisierung eingestellt. 
 *  Ist keine Taste gedrückt, wird der Treiber abgeschaltet.
 *  
 *  Die Bezeichnungen für die Richtungen im Schaltplan 
 *  und in der Pinbelegungs-Tabelle können abweichen.
 */
 
#include <SPI.h>


/*
 * SPI interrupt routine
 * Called whenever a SPI exchange has been done
 * The value obtained is in the data register
 * 
 * Communication is done via global variables:
 */
volatile byte spiFlag = 0;          // to tell main loop new data arrived
volatile byte spiErrflag = 0;
volatile byte spiGotval = 0;            // the value just received

// SPI interrupt routine
ISR (SPI_STC_vect)
{
    if (bitRead(SPSR, WCOL))    // write collision?
        spiErrflag = 1;         // yes
    spiGotval = SPDR;           // grab byte from SPI Data Register
    if (!spiFlag) {
      SPDR = spiGotval;           // and return this value by next transfer 
      spiFlag = 1;                // tell we got new byte    
    }
}  // end of interrupt routine SPI_STC_vect

/* Port numbers (in Arduino numbering) */

const char version [] = "1.0j";

// Magnet 1 (West-Ost)
const int m1_pwm = 3;       
const int m1_dir = 4;
const int m1_enable = 5;
// Magnet 2
const int m2_pwm = 9; 
const int m2_dir = 8;
const int m2_enable = 7;


// control pins (user interface)
const int speed_poti = A0;
const int key_1 = A2;
const int key_2 = A3;
const int key_3 = A4;
const int key_4 = A5;

// directions plus activation bit
#define DirNorth 4
#define DirWest 5
#define DirSouth 6
#define DirEast 7

// some globals
int magforce = 0;       // pwm ratio


/*
 * Setup
 */

void setup() {
    // initialize serial communications:
    Serial.begin(115200); 
    Serial.println();
    Serial.print("\nTheseus Magnets ");
    Serial.print(version);
    Serial.print(" Compiled: ");
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);

    // Magnet 1 control pins
    pinMode(m1_pwm, OUTPUT);        // Speed control
    pinMode(m1_dir, OUTPUT);
    pinMode(m1_enable, OUTPUT);
    digitalWrite(m1_enable, !LOW);

    // Magnet 2 control pins
    pinMode(m2_pwm, OUTPUT);        // Speed control
    pinMode(m2_dir, OUTPUT);
    pinMode(m2_enable, OUTPUT);
    digitalWrite(m2_enable, !LOW);

    // setup analog pins as key inputs
    pinMode(key_1, INPUT_PULLUP);
    pinMode(key_2, INPUT_PULLUP);
    pinMode(key_3, INPUT_PULLUP);
    pinMode(key_4, INPUT_PULLUP);

    // SPI Pin modes:
    pinMode(MOSI, INPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SS, INPUT);
    // Enable SPI in slave mode and enable interrupts
    bitClear(SPCR, MSTR); 
    bitSet(SPCR, SPE);
    SPI.attachInterrupt();


    // Init done
    Serial.println("Init done");

}

/* 
 *  Set the PWM values, directions and enable as required
 *  
 */
void setports(int force, int m1en, int m1dir, int m2en, int m2dir) {
    
    digitalWrite(m1_enable, !m1en);
    digitalWrite(m1_dir,  m1dir);       
    analogWrite(m1_pwm, force);
   
    digitalWrite(m2_enable, !m2en);
    digitalWrite(m2_dir,  m2dir);       

    analogWrite(m2_pwm, force);
}


void setdir(int direction) {
    switch (direction) {
    case DirNorth:
        setports(magforce, LOW, LOW, HIGH, LOW); 
        break;
    case DirWest: 
        setports(magforce, HIGH, LOW, LOW, LOW); 
        break;   
    case DirSouth:
        setports(magforce, LOW, LOW, HIGH, HIGH); 
        break;
    case DirEast:
        setports(magforce, HIGH, HIGH, LOW, LOW); 
        break;
    default:
        setports(magforce, LOW, LOW, LOW, LOW);
        break;
    }
}

void setbydir(int direction) {
    int mfsav = magforce;
    magforce = mfsav / 3;
    setdir(direction);
    delay(50);
    magforce = magforce + magforce;
    setdir(direction);
    delay(50);
    magforce = mfsav;
    setdir(direction);
}

// bit masks for keys
#define K1 1
#define K2 2
#define K3 4
#define K4 8
/*
 * Drive shield
 */

 // key values are essentially 4 Bits, so there are 16 cases;
void actuate(int key, int speed) {     
    switch (key) {   
        // single keys
        case K1: // Nord
            setbydir(DirNorth);
            break;
        case K2: // West
            setbydir(DirWest); 
            break;   
        case K3: // Süd
            setbydir(DirSouth); 
            break;
        case K4: // Ost
            setbydir(DirEast);
            break;     
            
        default:
            // if no key or invalid key combination, disable
            digitalWrite(m1_enable, !LOW);  
            digitalWrite(m2_enable, !LOW);  
            return;
    }
 
}

/*
 * tell which key(s) are pressed
 */
int getkey() {
    int rc = 0;
    if (digitalRead(key_1) == LOW) rc |= K1;
    if (digitalRead(key_2) == LOW) rc |= K2;
    if (digitalRead(key_3) == LOW) rc |= K3;
    if (digitalRead(key_4) == LOW) rc |= K4;
    return rc;
}

/* 
 *  Processing loop
 *  determine key pressed,
 *  then read speed from analog input
 *  
 */
void loop() {
    // check if new SPI command arrived
    if (spiFlag == 1) {
        
        Serial.print("SPI: ");
        Serial.print(spiGotval);
        switch (spiGotval) {
           case 0: 
              Serial.println(" Off");
              break;
           case DirNorth:
              Serial.println(" North");
              break;
           case DirSouth:
              Serial.println(" South");
              break;
           case DirWest:
              Serial.println(" West");
              break;
           case DirEast:
              Serial.println(" East");
              break;
        }
        setbydir(spiGotval);
        spiFlag = 0;       // command accepted
        return;
    }
    
    
    // check if key has changed
    static int lastkey = 0;
    int newkey = getkey();
    if (lastkey != newkey) {
        // key change, wait for debounce and try again
        lastkey = newkey;
        Serial.print("Key: ");
        Serial.println(newkey);
        delay(50);
        // now act upon key
        actuate(newkey, magforce);   
        return;
    }
    
    // get speed value as potentiometer value
    // ensure duty time between 1% and 90%
    magforce = map(analogRead(speed_poti), 0, 1023, 3, 252);
    static int prevforce = 0;
    if (abs(magforce - prevforce) > 3) {       
        Serial.print("Speed: ");
        Serial.print(magforce);
        Serial.print(" ");
        Serial.print(magforce*100/255);
        Serial.println("%");
        prevforce = magforce;
    }
 
}



            
        
