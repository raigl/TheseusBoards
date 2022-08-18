/*
    Theseus SPI Master
    
    Für die Details der SPI-Protkolle siehe das Dokument "Theseus-Aufbau"

    Durch Eintreffen eins Zeichen von der seriellen Schnittstelle wird der
    Diagnose-Modus aktiviert, in dem die folgenden
    Kommandos von der seriellen Schnittstelle ausgeführt werden.
    Abbruch von Kommandos oder -Sequenzen durch Hard-Reset.

    Beispiels-Testsequenz: goas1o1o1tt1n1o1d
    oder: goas1t111att1n1o2tta1s1o1n1o1d
    Laufweg: geas2o1n1o2s1o1n1d

    Testversion: Steuerung über serielle Schnittstelle
        n	Nord als nächste Richtung setzenw1
        e o	Ost als nächste Richtung setzen
        s	Süd als nächste Richtung setzen
        w	West als nächste Richtung setzen
        d	Magnete deaktivieren
        a	Magnete aktivieren
        p	Position der Motore abfragen
        q	Status der Motore abfragen
        r	Position der Reed-Kontakte für die Mausposition abfragen
        c	Beende eine Aktion (Motorbewegung) vorzeitig (cancel)
        1, 2, 3, 4
            Bewegung bis 1, 2, 3 bzw. 4. Feldmitte
        t   Um 90° drehen (turn)
        f   Um 180° drehen (flip)
        .   Wartezeit 0.5sec
        ,   Warte, bis Bewegung beendet
        ;   Warte nach Zielkontakt
        m   Mittenposition sicherstellen
        g   goto mouse: positioniere die Magnete unter die Maus
        =   Setze Speicher auf Anfangswerte (derzeit Osten)
        [v   Drehrichtung ändern (vice versa) ]
        x   Exit to eXplore mode

    Die Richtungscodes speichern die Richtung für nachfolgene Bewegungen ab.
    Sind die Magnete aktiviert, wird die neue Richtung wirksam,
    und es wird gewartet, um die Drehung abzuschliessen.
    
    Drehungen um 180° sind möglich und werden durch einen kurzen 90°-Impuls 
    eingeleitet.

    Deaktivieren der Magnete (mit 'd') schaltete sie sofort aus (ohne anschließende
    Wartezeit);
    aktivieren (mit 'a') aus dem deaktivierten Zustand schaltet in der gemerkten
    Richtung ein, wartet kurz und läßt sie eingeschaltet.
    Die Magnete sind nach einem Reset deaktiviert.

    Eine Drehung (turn) ändert die aktuelle Richtung um 90°,
    schaltet die Magnete ein und wartet kurz,
    unabhängig davon, ob die Magnete aktiviert sind oder nicht.
    
    Die Drehrichtung (Uhrzeigersinn = cw, entgegen = ccw) ist derzeit
    nicht im Kommandomodus veränderbar, siehe Macro TURN_MOUSE

    Eine Bewegung (1, 2, 3 oder 4) verschiebt die Magnete auf die
    nächste, übernächste usw. Feldmitte.
    Sofern die Magnete aktiviert sind, wird bei einer Ziel- oder Zaunberührung
    angehalten, und die Magnete werden automatisch deaktiviert;
    ohne Ziel- oder Zaunberührung bleiben die Magnete eingeschaltet,
    Es wird zudem automatisch auf das Ende der Bewegung gewartet.
    Sind die Magnete deaktiviert, werden sie unabhängig vom Ziel- oder
    Zaunkontakt verschoben, also auch, wenn die Maus noch Zielkontakt hat;
    zudem wird nicht auf das Ende der Bewegung gewartet.

    Das Wartekommando ',' wartet auf das Ende einer aktuellen Bewegung;
    es ist nach einer Bewegung bei ausgeschaltetem Magnet zu verwenden
    und erlaubt den Test von Abfragen während einer Bewegung.
    Die Magnete werden nicht verändert.

    Das Kommando '.' wartet kurz (0.5sec) und wird bei aktivierten
    Magneten automatisch ausgeführt. 
    
    Das Kommando 'p' fragt die Position der Magnet ab und gibt sie aus;
    ein Fragezeichen ('?') steht für eine unbekannte Position.
    Die Position wird oktal als 0xy (x: Ost-West, y: Nord-Süd) ausgegeben.
    
    Das Kommando 'g' (goto mouse) deaktivert die Magnete.
    Ist keine gültige Magnetposition gegeben, wird sofort abgebrochen.
    Ist keine gültige Mausposition vorhanden, wird gewartet, bis eine solche
    vorliegt (Abbruch nur durch RESET);
    sodann werden die Magnete an die Mausposition verschoben
    und auf das Ende der Bewegungen gewartet.
    Die Magnete werden nicht automatisch aktiviert;
    meist folgt ein 'oad', um die Maus nach Osten zu normieren.
    Beachte, dass die Maus in Ost-West-Richtung kein Positionssignal
    erzeugt, also insbesondere nicht in West-Richtung, so dass kein
    Umklappen um 180° zu befürchten ist.

    Mit dem Kommando 'x' wird der Testmodus verlassen 
    und der Suchmodus gestartet, ohne jedoch die gespeicherten
    Richtungen zu initialisieren.

    Versionen: 
    1.3v  Erste Version für HNF
    1.4a  Drehung im Uhrzeigersinn, Anfangswerte nach Osten
    1.5a  Drehung 180° mit 120ms Vorimpuls 
    2.0a  Überarbeitung wg. Zielmodus (goal mode)
   
    Ist die SPI Taktrate 500kHz, dann dauert eine Übertragung 16µs (8 bit).
    SPI.transfer() startet die Übertragung und wartet auf das Ende,
    um den empfangenen Wert zurückzugeben. Dieser wurde jedoch am Ende
    des vorherigen Tansfers bereitgestellt. 
    Bis der aktuelle Wert bereitsteht, ist eine Wartezeit notwendig,
    bis im Slave der Interrupt bearbeitet wurde. 
    Bei einer Positions-Abfrage ist dies an dem MOTOR_StatusPositionBit
    erkennbar.
    

*/

const char version [] = "2.1d";

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
// Wartezeit: mind. 8/SPI_speed, > 16                 ms
#define SPI_WAIT_RESPONSE 32
#define SPI_speed 500000               // SPI clock speed < 4 MHz

// Drehrichtung der Maus (nicht per Tastatur-Kommando einstellbar)
#define TURN_MOUSE(dir) ((dir + 3) % 4) // +1: CCW +3:CW
#define FLIP_MOUSE(dir) ((dir + 2) % 4) // any wiseness

// Wartezeit für Vierteldrehung mit Ausschwingen
#define MOUSE_TURN 300
// Wartezeit für Anstoßen einer 180° Drehung
#define MOUSE_PUSH 150
// Wartezeit Aus- oder erneutes Einschalten in derselben Richtung
#define MOUSE_NOTURN 20


// chip select output pins for slaves
const int chipSelectMagnets = 5;
const int chipSelectMotors = 4;
const int chipSelectMatrix = 6;

// Beeper pin
const int beeper = 3;

void beep() {
    digitalWrite(beeper, HIGH),
    delay(500);
    digitalWrite(beeper, LOW);
}

// Interner Fehler
void (*Reset_C)(void) = 0;
void internalError(const char* msg) {
  Serial.print("** Internal error: ");
  Serial.println(msg);
  setMagnetOff();
  for (int i = 1; i <= 5; ++i) {
    beep();
    delay(200);
  } 
  delay(5000); 
  Reset_C();
} 

/*
    Codes für die Kommunikation mit Magneten und Motoren
*/

// The include file is a symbolic link as the IDE does not support relative paths
#include "SPI_Motor.h"
#define posValid(pos) (MOTOR_PosX(pos) != 0 && MOTOR_PosY(pos) != 0)

char dirCode[5] = { 'N', 'W', 'S', 'E', 'X'};
int lastMagnetDir = 0;          // 0=N, 1=E, 2=S, 3=W
bool magnetsActive = false;

/*
    Magnet setzen via SPI-Kommando
*/

/*
  Einschalten mit Richtung 0..3,
  Ausschalten mit 4 oder größer, dann bleibt lasstMagnetDir unverändert.
  Gibt den vorherigen Status vollständig zurück (mit Bit 2)
  
  Nicht direkt verwenden;
  benutze setMagnetOn(dir) und setMagnetOff() 
*/
byte setMagnetSPI(byte dir) {
  byte oldval;
  
  // Start SPI for magnets
  digitalWrite(chipSelectMagnets, HIGH);
  delayMicroseconds(16);  
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE0));
  
  // prepare command
  if (dir < 4)
    dir = dir | 4;
  else
    dir = dir & 03;
  oldval = SPI.transfer(dir);
  // Wartezeit sollte nicht notwendig sein
  delayMicroseconds(16);            
  
  // disable 
  digitalWrite(chipSelectMagnets, LOW);
  delayMicroseconds(16);  
  SPI.endTransaction();

  // Rückgabe: vorheriger Status
  return oldval;
}


/*
  Einschalten von Magneten

  Drehung um 180° wird durch Vorimpuls 90° bewirkt.
  Wenn es die Richtung in lastMagnetDir ist, 
  wird kürzer gewartet, da keine Drehung erfolgt
  
  Setzt lastMagnetDir (bitte nur lesen)
*/
void setMagnetOn(byte dir) {
  Serial.print("Magnet: ");

  // Drehung um 180°
  if (abs(dir - lastMagnetDir) == 2) {
    Serial.print("Auto-Turn ");
    Serial.print(lastMagnetDir);
    Serial.print(" ");
    Serial.print(dir);
    Serial.print(" ");
    Serial.println((dir+1) % 4);
    setMagnetSPI(  (dir+1) % 4);
    delay(MOUSE_PUSH);
    }

  bool samedir = (dir == lastMagnetDir);
  setMagnetSPI(dir);                   
  lastMagnetDir = dir;

  if (samedir) {
    delay(MOUSE_NOTURN);
    Serial.print(" == ");
  } else {
    delay(MOUSE_TURN);
  }
  Serial.print(" ");
  Serial.print(dir);
  Serial.print(": ");
  Serial.println(dirCode[dir]);
}

/*
  Magnet ausschalten, 
  lastMagnetDir bleibt erhalten
*/
void setMagnetOff() {
  setMagnetSPI(4);      // >3 ist ausschalten
  delay(MOUSE_NOTURN);
  Serial.println("Magnet off.");
  
}

/** 
      Motor-Interface
**/


/*
    Motor im Modus 'cmd' bewegen in Richtung 'direction' um 'count' Felder.
    Es wird nicht auf das Ende des Befehls gewartet.
    Die vorherige Bewegung muss beendet sein.
*/
unsigned maxCmdRepeat = 0;
unsigned overrunCount = 0;

void moveMotor(byte cmd, byte direction, byte count ) {
  byte statval;
  
  // SPI begin
  digitalWrite(chipSelectMotors, HIGH);
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE0));
  
  unsigned long now = 0;
  // Warte auf Ready
  do {
    statval = SPI.transfer(MOTOR_CmdStatus);
    delayMicroseconds(SPI_WAIT_RESPONSE);
    if (bitIsSet(statval, MOTOR_StatusReadyBit))
      break;
    if (now == 0) {
      now = millis();
      Serial.print(" --- moveMotor not ready, stat=");
      Serial.print(statval, BIN);
    }
  } while (true);
  if (now != 0) {
    Serial.print(" ");
    Serial.print(millis() - now);
    Serial.println("ms");
  }
      
  

  // Kommando senden
  statval = SPI.transfer(MOTOR_SPIcommand(direction, count, cmd));

  
  // Der Status muss Ready sein
  if (bitIsClear(statval, MOTOR_StatusReadyBit)) {
    Serial.print("Error, old status not ready:");
    Serial.println(statval, BIN);
    internalError("old status not ready");
  }
  
  // auf Overrun und not-Ready prüfen
  now = 0;
  do {
    delayMicroseconds(SPI_WAIT_RESPONSE);
    statval = SPI.transfer(MOTOR_CmdStatus);
    
    if (bitIsSet(statval, MOTOR_StatusPositionBit))
      continue;                 // alte position, nochmal
      
    if (bitIsSet(statval, MOTOR_StatusOverrunBit)) {
      // Kommando nochmal senden, weil verworfen
      statval = SPI.transfer(MOTOR_SPIcommand(direction, count, cmd));
      overrunCount += 1;
      continue;
    }
    // wenn kein Ready-Bit, fertig
    if (bitIsClear(statval, MOTOR_StatusReadyBit))
        break;
        
    // zeit bestimmen
    if (now == 0) 
      now = millis();
  } while(true);
  
  if (now != 0) {
    Serial.print("response delayed ");
    Serial.println(millis() - now);
  }
  
    
  // SPI deselect
  digitalWrite(chipSelectMotors, LOW);
  delayMicroseconds(SPI_WAIT_RESPONSE);
  
  

}


/*
    Motor anhalten
    Es wird nicht auf eine Statusänderung gewartet.
*/
void stopMotor() {
  // select
  digitalWrite(chipSelectMotors, HIGH);
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE0));
  
  byte statval = SPI.transfer(MOTOR_SPIcommand(0, 0, MOTOR_CmdStop));

  delayMicroseconds(SPI_WAIT_RESPONSE);
  
  // deselect
  digitalWrite(chipSelectMotors, LOW);
  SPI.endTransaction();
}

/*
    Im Motortreiber Normierung auf Feldmitte starten
*/
void centerMotor() {
    moveMotor(MOTOR_CmdCenter, 0, 0);
}


/*
    Motor Status abfragen
    Resultat ist aktueller Status
*/

byte getMotorStatus() {
  byte statval;
  // enable
  digitalWrite(chipSelectMotors, HIGH);
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE0));
  
  statval = SPI.transfer(MOTOR_CmdStatus);      // liefert alten Status
  delayMicroseconds(SPI_WAIT_RESPONSE);         // Zeit für Interrupt-Reaktion
  statval = SPI.transfer(MOTOR_CmdStatus);      // liefert aktuellen Status
  
  // Falls die Antwort eine Position ist, wiederholen
  while (bitIsSet(statval, MOTOR_StatusPositionBit)) {
    delayMicroseconds(SPI_WAIT_RESPONSE);   
    statval = SPI.transfer(MOTOR_CmdStatus);                     
  }
  
  // SPI disable 
  digitalWrite(chipSelectMotors, LOW);
  SPI.endTransaction();
  return statval;
}

/*
    Motor Position abfragen. 
    Resultat ist die Position in jeweils 3 bit
    
    Die Abfrage wird mit verlängerten Wartezeiten solange
    wiederholt, bis das Positonsbit erscheint.
    
    Andernfalls wird die ungültige Position 66 zurückgegeben.
    (077 ist reserviert, falls die SPI-Verbindung gestört ist.)
     
*/
unsigned maxPositionResponseDelay = 0;

byte getMotorPosition() {
  byte posval;
  // enable
  digitalWrite(chipSelectMotors, HIGH);
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE0));
  
  
  unsigned response_delay = SPI_WAIT_RESPONSE;
  // Positionsabfrage gefolgt von Statusabfrage
  do {
    posval = SPI.transfer(MOTOR_CmdPosition);   // Position anfragen, liefert Status
    delayMicroseconds(response_delay);          // Zeit für Interrupt-Reaktion
    posval = SPI.transfer(MOTOR_CmdStatus);     // Statusanfrage liefert Position 
    if (bitIsSet(posval, MOTOR_StatusPositionBit))
       break;
    delayMicroseconds(response_delay); 
    response_delay += response_delay;           // exponentielle Verlängerung
    posval = 066;                               // falls abgebrochen
  } while (response_delay < 10000 );            // max. 10ms
       
  if (response_delay > maxPositionResponseDelay)
    maxPositionResponseDelay = response_delay;
  
  // SPI disable
  digitalWrite(chipSelectMotors, LOW);
  SPI.endTransaction();
  return posval & 077;   // nur die Position
}

/*
   Bestimmt eine Position der Maus anhand der Reed-Kontakte
   als 3+3 Bit Wert: Bits 0..2 für West-Ost, 3..5 für Nord-Süd,
   d.h. oktal 11 für Nord-West, 51 für Süd-West, 15 für Nord-Ost
*/
int readMatrix() {
  unsigned char result[8];
  int i, byt;
  unsigned char mask;
  // start 8 byte read
  digitalWrite(chipSelectMatrix, HIGH);
  SPI.beginTransaction(SPISettings(SPI_speed, LSBFIRST, SPI_MODE0));
  // TODO: use SPI.transfer(buffer, size)
  for (i = 0; i < 8; ++i) {
    result[i] = SPI.transfer(0);
    // Wartezeit sollte nicht notwendig sein, Hardware-Lösung
    delayMicroseconds(8);
  }
  // done
  digitalWrite(chipSelectMatrix, LOW);
  SPI.endTransaction();

  // Reed-Kontakte

  mask = 1;
  for (i = 0; i < 32 ; ++i ) {
    byt = 4 + i / 8;
    if ((result[byt] & mask) != 0) {
      int v = (31 - i) / 5;
      int u = (31 - i) % 5;
      return 8 * (v + 1) + (u + 1);
    }
    mask = mask * 2;
    if (mask == 0)
      mask = 1;
  }
  return 0;
}


/*
    Motor Status ausgeben
    Achtung: Das Ziel- und Zaunbit wird durch die Abfrage
             NICHT aktualisiert
*/
void printMotorStatus() {
  Serial.print("Motor Status: ");
  byte stat = getMotorStatus();

  if (bitIsSet(stat, MOTOR_StatusMidBit))
    Serial.print("M");    
  if (bitIsSet(stat, MOTOR_StatusLimitBit))
    Serial.print("L");
  if (bitIsSet(stat, MOTOR_StatusGoalBit))
    Serial.print("G");
  if (bitIsSet(stat, MOTOR_StatusFenceBit))
    Serial.print("F");
    
  if (bitIsSet(stat, MOTOR_StatusReadyBit))
    Serial.print(" Ready");
  else
    Serial.print(" Busy");
   
  // SPI-Statistiken
  if (overrunCount > 0) {
    Serial.print(" Overruns: ");
    Serial.print(overrunCount);
  }
  if (maxCmdRepeat > 0) {
    Serial.print(" Repeats: ");
    Serial.print(maxCmdRepeat);
  }
  Serial.println();
}

/*
    Zeige die Motor-Position an.
    Das Positionsbyte enthält x in Bit 3..5, den Ost-West-Wert,
    und y in Bit 0..2, den Nord-Süd-Wert, (beides von 1 bis 5),
    also ist 15 die linke untere Ecke, die Süd-West-Ecke.
*/
void printMotorPosition() {
  Serial.print("Motor Position: ");
  byte pos = getMotorPosition();
  Serial.print(pos, OCT);
  if (maxPositionResponseDelay > SPI_WAIT_RESPONSE) {
    Serial.print(" response delay=");
    Serial.print(maxPositionResponseDelay);
  }
  Serial.println();
}

/*
   Warte, bis Bewegung beendet, und gib Status zurück
   Fehlerbits werden ignoriert
*/
byte waitMove() {
  byte mostat;
  do {
    mostat = getMotorStatus();
  } while (bitIsClear(mostat, MOTOR_StatusReadyBit));
  return mostat;
}

/*
    Warte, bis eine Maus erkannt wird, und positioniere dorthin.
    
    Nicht beliebig durch Konsole unterbrechbar.
    Auf Ende der Bewegungen wird gewartet.
*/
#define MouseStableTime 300 // ms

void locateAndGo() {
  byte mostat;
  
  // Magnete aus
  magnetsActive = false;
  setMagnetOff();

  // Alle Eingaben verwerfen
  while (Serial.available())
      Serial.read();

  // Motor auf Feldmitte positionieren
  centerMotor();
  mostat = waitMove();
  if (bitIsClear(mostat, MOTOR_StatusMidBit)) {
    Serial.print(mostat, BIN);
    Serial.print(" ");
    printMotorStatus();
    internalError("Center failed");
  }
  
  // Motorposition ermittlen
  byte mpos = getMotorPosition();
  Serial.print("Locate Motor:");
  Serial.print(mpos, OCT);
  
  // prüfe gültige Position
  if (!posValid(mpos)) {
    Serial.println(" -- bad motor position");
    return;  
  }

  // auf Mausposition stabil warten
  byte rpos;
  Serial.print(" -- waiting for mouse ... ");
  unsigned long degli = millis();
  while ((millis() - degli) < MouseStableTime) {
     rpos = readMatrix();
     if (rpos == 0)
        degli = millis();  
     // abbrechen wenn Kommandoeingabe
     if (Serial.available())
        return;   
  }
  Serial.print(rpos, OCT);
  Serial.println(" ok.");
  // beep();                   // signal: mouse detected
  // delay(500);

  
  Serial.print(" Matrix: ");
  Serial.println(rpos, OCT);
  if (rpos == 0) return false;

  int xdiff = MOTOR_PosX(rpos) - MOTOR_PosX(mpos);
  Serial.print(" xdiff = ");
  Serial.print(xdiff);
  int ydiff = MOTOR_PosY(rpos) - MOTOR_PosY(mpos);
  Serial.print(" ydiff = ");
  Serial.print(ydiff);
  Serial.println();

  // Die folgenden Teile sind nicht alternativ, daher waitMove()
  if (xdiff > 0 && xdiff < 5) {
    Serial.print(" moveMotor East ");
    Serial.print(xdiff);
    moveMotor(MOTOR_CmdMove, MOTOR_DirEast, xdiff);
    waitMove();
  }
  if (xdiff < 0 && xdiff > -5) {
    Serial.print(" moveMotor West ");
    Serial.print(-xdiff);
    moveMotor(MOTOR_CmdMove, MOTOR_DirWest, -xdiff);
    waitMove();
  }
  if (ydiff > 0 && ydiff < 5) {
    Serial.print(" moveMotor South ");
    Serial.print(ydiff);
    moveMotor(MOTOR_CmdMove, MOTOR_DirSouth, ydiff);
    waitMove();
  }
  if (ydiff < 0 && ydiff > -5) {
    Serial.print(" moveMotor North ");
    Serial.print(-ydiff);
    moveMotor(MOTOR_CmdMove, MOTOR_DirNorth, -ydiff);
    waitMove();
  }
  
  // Prüfen
  mpos = getMotorPosition();
  mostat = getMotorStatus();
  Serial.print(" done, status:");
  Serial.print(mostat, BIN);
  Serial.print("  at ");
  Serial.println(mpos, OCT);
  if (mpos != rpos)
    internalError("Motor pos fault");  // mit Neustart
  // 
  return true;
}




/*
  Speicher für die Ausgangs-Richtungen per Position
  Der Einfachheit halber werden anstelle von 25*2 Bit
  64 Byte verwendet, die mit der Position, wie sie 
  vom Motortreiber geben wird, indiziert werden
  Initialisierung in setup()
*/
byte exitDirs[64]; 


/**
  Gemeinsame Strategie für Such- und Zielmodus.
  
  Im Suchmodus wird sogleich beim Betreten eines Feldes gedreht,
  im Zielmodus hingegen nur bei Wandberühung. 
  
  Das Unterprogramm bewirkt die nächste Bewegung; 
  solange noch eine Bewegung aktiv ist, wird sofort zurückgekehrt.
  Solange noch kein Ziel erreicht ist, wird 'false' zurückgegeben, 
  und es kann sofort -- nach Auswertung anderer Ereignisse --
  wieder aufgerufen werden.
  
  Mit dem Parameter init werden nur die globalen Variablen derart zurückgesetzt,
  dass kein Feldeintritt verwendet wird.
  Achtung: der zielModus wird nicht verändert; dies muss separat erfolgen.
  
  Ein fehlender Randzaun wird als Ziel angesehen, 
  damit die Suche sofort beendet wird.
  
  Bei erfolgreichem Ende (return true) ist die letzte Bewegung 
  (Rückkehr zur Mitte) nicht unbedingt abgeschlossen;
  ein 'waitMove()' ist normalerweise notwendig.
      
**/

// globale Variable für die Suchstrategie
bool zielModus = false;
byte oldpos = 0;
int fieldcount = 0;                 
byte lockdir = MOTOR_DirInvalid;    // Gegenrichtung zu der Eingangsrichtung


/**
  Hilfsroutinen
**/

/*
  Maus umdrehen und zurück zur Mitte bewegen
*/
void backToCenter() {
    setMagnetOn(FLIP_MOUSE(lastMagnetDir));   
    moveMotor(MOTOR_CmdMove, lastMagnetDir, 0);  // keine Zaunberührung auswerten 
    delay(200);
    Serial.println("Back to center.. ");
}

/* 
  Anzahl der Felder gleicher Richtung bestimmen
*/
int countSameDirection(byte pos) {
  int dir;
  int steps = 0;        
  do {
    ++steps; 
    dir = exitDirs[pos];      // bisherige Richtung
                 
    // nächste Position im Speicher
    switch(dir) {
      case MOTOR_DirNorth:
        if (MOTOR_PosY(pos) < 2)
          return steps;
        pos -= 1;
        break;
      case MOTOR_DirSouth:
        if (MOTOR_PosY(pos) > 4)
          return steps;
        pos += 1;
        break;
      case MOTOR_DirWest:
        if (MOTOR_PosX(pos) < 2)
          return steps;
        pos -= 8;
        break;
      case MOTOR_DirEast:
        if (MOTOR_PosX(pos) > 4)
          return steps;
        pos += 8;
        break;
      }
      // Nochmal, wenn im nächsten Feld die gleiche Richtung
    } while (dir == exitDirs[pos]);
  // found no further matching direction
  if (steps > 4) {
      Serial.print("Too many steps: ");
      Serial.println(steps);
      steps = 1;
  } else {
      Serial.print("Steps: ");
      Serial.println(steps);
  }

  return steps;
}

/*
  Gespeichterte Positionen alle auf Anfangswert
*/
void resetExitDirs() {
  for (int i=0; i<64; ++i)
    exitDirs[i] = MOTOR_DirEast;
}

/* 
  Zu Anfang oder nach Erreichen des Ziels: 
  warten, bis die Maus entfernt und wieder eingesetzt wurde.
*/

void restartMouse() {
  Serial.println("Restart Mouse");
  
  // warte, dass keine Maus vorhanden ist
  int msgcnt = 0;
  while (readMatrix() != 0) {
    if (msgcnt % 50 == 1) {
      Serial.print("Remove Mouse to start from ");
      Serial.println(readMatrix(), OCT);
    }
    ++msgcnt;
    delay(100);
    if (Serial.available() > 0)
      return; 
  }
  
  // Suche Maus
  locateAndGo();
  
  // Maus fangen
  lastMagnetDir = MOTOR_DirWest;
  setMagnetOn(MOTOR_DirWest);
  delay(MOUSE_TURN);
  // setMagnetOff();
    
  lockdir = MOTOR_DirInvalid;
  oldpos = getMotorPosition();
  return true; 
  
}


/*
  Strategie neu starten
  Parameter ist 'false' für Such- und 'true' für Ziel-Strategie
*/
void initFindGoal(bool modus) {
  if (modus)
    Serial.println("--- start goal mode:");
  else
    Serial.println("--- start exploration mode:");

  oldpos = getMotorPosition();      // es gibt keine Eintrittsrichtung
  lockdir = MOTOR_DirInvalid;
  fieldcount = 0;
  zielModus = modus;
  restartMouse();  
  Serial.println("----------");
}

/**
  Suchstrategie
**/

/*
  Zielsuche im Such- oder Zielmodus
  
  Bei jedem Aufruf wird entweder nichts getan,
  wenn noch eine Bewegung aktiv ist,
  oder ansonsten die nächste Bewegung angestoßen.
  
  Wurde ein Zaun erreicht, wird umgedreht
  und die Rückwärtsbewegung zur Mitte begonnen;
  zudem wird die abgespeicherte Ausgangsrichtung
  weitergeschaltet; hierzu muss die letze gültige Position
  in oldpos gemerkt werden.
  
  Wurde das Ziel erreicht, wird die Maus stehen gelassen,
  aber die Motor in eine Feldmitte bewegt.
  
  Sind weder Zaun noch Ziel aktiv, ist der Motor in Feldmitte.
  
  Falls ein Feldwechsel erkannt wird (wenn kein Zaun getroffen wurde),
  wird im Suchmodus die gespeicherte Ausgangsrichtung weitergeschaltet
  und die Ausgangsrichtung einmal gesperrt.
  
  Sodann wird um ein (Suchmodus) oder meherer (Zielmodus)
  Feld in der -- ggf. im Suchmodus geänderten -- gespeichterten 
  Richtung bewegt.   
  
*/

void findGoal() { 
  byte outdir;
  byte stat;
    
  // Motor-Status ermitteln
  stat = getMotorStatus();
  
  // Bewegung abgeschlossen?
  if (bitIsClear(stat, MOTOR_StatusReadyBit)) 
    return;                 
  // letztes Kommando ist somit fertig
  
  // Am Zaun oder Grenze?
  if (bitIsSet(stat, MOTOR_StatusFenceBit) || bitIsSet(stat, MOTOR_StatusLimitBit)) {
    Serial.print("Fence (or border) reached, oldpos=");
    Serial.println(oldpos, OCT);
    exitDirs[oldpos] = TURN_MOUSE(exitDirs[oldpos]);
    backToCenter();             // Maus mit Magnet zurück zur Mitte
    waitMove();
    return;                    
  }
  
  // Ziel erreicht? 
  if (bitIsSet(stat, MOTOR_StatusGoalBit)) { 
    Serial.println("++++++ Goal reached");
    beep();
    // delay(200);
    // beep();
    printMotorStatus();
    // Maus stehen lassen, Magnete abschalten und zurück in Feldmitte
    setMagnetOff();
    centerMotor();                // nur den Motor
    waitMove();
    // Weiter im Zielmodus
    initFindGoal(true); 
    return;                  
  }

  // Motor muss in Feldmitte stehen 
  if (bitIsClear(stat, MOTOR_StatusMidBit)) {
    printMotorStatus();
    internalError("Motor not centered");
  }
  
  // Gültige Position prüfen
  byte pos = getMotorPosition();
  // Muss hier immer gültig sein, Zaunkontakt ist schon behandelt
  if (!posValid(pos)) {
    Serial.print("Invalid position: ");
    Serial.println(pos, OCT);
    printMotorStatus();
    internalError("Invalid Position");
  }
      
  // Abbruch Zielmodus ("discouragement"), wenn mehr als 24 Felder besucht wurden
  if (zielModus && fieldcount > 24) {
    Serial.println("Goal -> Exploration mode");
    zielModus = false;
    fieldcount = 0;
    return;                 // Aufrufer kann Änderung auswerten
  }
  
  // Gespeichterte Richtung holen
  outdir = exitDirs[pos];
  
  // Feldwechsel?
  if (pos != oldpos ) {
    Serial.print("Feldwechsel ");
    Serial.print(oldpos, OCT);
    Serial.print(" -> ");
    Serial.println(pos, OCT);
    oldpos = pos;
    fieldcount++;
    
    // Ausgangsrichtung einmal sperren 
    lockdir = FLIP_MOUSE(lastMagnetDir);

    // im Suchmodus bei Feldwechsel zusätzlich drehen
    if (!zielModus) 
      outdir = TURN_MOUSE(outdir);

  }
  
  // Zunächst nicht über den Eingang wieder verlassen;
  // das vermeidet auch 180° Drehungen
  if (lockdir == outdir) {
    Serial.print("locked, ");
    Serial.print(dirCode[lockdir]);
    outdir = TURN_MOUSE(outdir);
    Serial.print(" new outdir=");
    Serial.println(dirCode[outdir]);
    lockdir = MOTOR_DirInvalid;
  } else {
    Serial.print("outdir=");
    Serial.print(dirCode[outdir]);
    Serial.print(" lockdir: ");
    Serial.println(dirCode[lockdir]);
  }
  // Richtung des Verlassens speichern
  exitDirs[pos] = outdir;
  
  // im Zielmodus möglichst um mehrere Felder bewegen
  byte moveCount = 1;
  if (zielModus) 
    moveCount = countSameDirection(pos);    // benutzt exitDirs[pos]


  // Bewegung anstoßen
  setMagnetOn(outdir);
  Serial.print("Drag -> " );
  Serial.print(dirCode[outdir]);
  Serial.print(" count=");
  Serial.println(moveCount);

  moveMotor(MOTOR_CmdDrag, outdir, moveCount);
  // Bewegung läuft, Ziel noch nicht gefunden
  return;
}

/**
  Interaktive Kommandos ausführen
**/

void processCommands() {
  if (Serial.available() < 1)
    return;
  char cmd = Serial.read();
  if (isControl(cmd))
    return;
  // if (!isAlphaNumeric(cmd))     return;
  Serial.print(cmd);
  Serial.print(" ");
  // read matrix
  if (cmd == 'r') {
    int pos = readMatrix();
    Serial.print("Sense Position: ");
    Serial.println(pos, OCT);
    return;
  }

  static byte nextDir = MOTOR_DirInvalid;
  // Bewegungsrichtung merken und ggf. Magnet einschalten:
  if (cmd == 'n') {
    nextDir = MOTOR_DirNorth;
    if (magnetsActive) 
      setMagnetOn(nextDir);
  }
  if (cmd == 's') {
    nextDir = MOTOR_DirSouth;
    if (magnetsActive) 
      setMagnetOn(nextDir);
  }  
  if (cmd == 'w') {
    nextDir = MOTOR_DirWest;
    if (magnetsActive) 
      setMagnetOn(nextDir);
  }  
  if (cmd == 'e' || cmd == 'o') {
    nextDir = MOTOR_DirEast;
    if (magnetsActive) 
      setMagnetOn(nextDir);
  }
  
  // Magnet einschalten: activate
  if (cmd == 'a') {
    magnetsActive = true;
    setMagnetOn(nextDir);
    delay(MOUSE_TURN);
  }
  // Magnet ausschalten: deactivate
  if (cmd == 'd') {
    magnetsActive = false;
    setMagnetOff();
  }
  
  byte mostat;

  // Bewegung um n Felder. Als nächstes warten und Status holen,
  // z.B. N1,qp
  if (cmd >= '0' && cmd <= '4') {
    int cnt = cmd - '0';
    if (magnetsActive) {
      setMagnetOn(nextDir);
      moveMotor(MOTOR_CmdDrag, nextDir, cnt);
    } else {
      setMagnetOff();
      moveMotor(MOTOR_CmdMove, nextDir, cnt);
    }
    return;
  }

  // Position auslesen
  if (cmd == 'p') {
    printMotorPosition();
    return;
  }
  
  // Status auslesen
  if (cmd == 'q') {
    printMotorStatus();
    return;
  }

  // Gehe zur Maus
  if (cmd == 'g') {
    locateAndGo();
    return;
  }

  // warte, bis Bewegung beendet
  if (cmd == ',') {
    Serial.print("Warten... ");
    while (bitIsClear(getMotorStatus(), MOTOR_StatusReadyBit)) {
      delay(100);
      Serial.print(".");
    }
    Serial.println();
    printMotorStatus();
    printMotorPosition();
    return;
  }

  // Bewegung anhalten
  if (cmd == 'c') {
    Serial.println("Motor stop: ");
    stopMotor();
    waitMove();
    return;
  }

  // Zentriere auf Mittenposition
  if (cmd == 'm') {
    Serial.print("Center Motor: ");
    centerMotor();
    mostat = waitMove();
    Serial.print(mostat, BIN);
    Serial.print(" pos=");
    Serial.print(getMotorPosition(), OCT);
    if (bitIsClear(mostat, MOTOR_StatusMidBit))
      Serial.println(" ??? failed.");
    Serial.println();
    return;
  }

  // Maus um 90° drehen
  if (cmd == 't') {
    setMagnetOn(TURN_MOUSE(lastMagnetDir));
    return;
  }

  // Maus um 180° drehen
  if (cmd == 'f') {
    setMagnetOn(FLIP_MOUSE(lastMagnetDir));
    delay(MOUSE_TURN);
    return;
  }
  // warten
  if (cmd == '.') {
    delay(MOUSE_TURN);
    return;
  }
  
  // Speicher initialisieren
  if (cmd == '=') {
    resetExitDirs();
    return;
  }
  
  // Explore-Mode
  if (cmd == 'x') {
    Serial.println("Explore mode...  ");  
    // remove any characters in buffer
    while (Serial.available())
      Serial.read();
    // Neustart im Suchmodus
    initFindGoal(false);        // im Suchmodus neu starten  
    return;
  }

}



/**
  Zentrale Schleife
**/
bool commandMode = false;
void processCommands();

void loop() {

  // Ein Eingabezeichen wechselt in den interaktiven Modus
  if (Serial.available())
    commandMode = true;
    
  // Kommando-Modus
  if (commandMode) {
    processCommands();            // ändert ggf. commandMode
    return;
  }
  
  //TODO: Hier Kommunikation mit dem Bedienteil einfügen
    
  // Such- und Zielmodus sind fast gleich;
  // wird gesteuert über globable Variabale
  findGoal();     
  
}
    
   
/**
  Initialisierung
**/
void setup() {
  // Serial I/O
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.print("SPI Master ");
  Serial.print(version);
  Serial.print(" Compiled: ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

  // SPI: alle deselektieren
  pinMode(chipSelectMagnets, OUTPUT);
  digitalWrite(chipSelectMagnets, LOW);
  pinMode(chipSelectMotors, OUTPUT);
  digitalWrite(chipSelectMotors, LOW);
  pinMode(chipSelectMatrix, OUTPUT);
  digitalWrite(chipSelectMatrix, LOW);
  
  // SPI starten
  SPI.begin();

  // TODO: check if SPI connections are working
  
  // Summer einrichten
  pinMode(beeper, OUTPUT);
  digitalWrite(beeper, LOW);

  // Speicher initialisieren
  resetExitDirs();
    
  // Magnete ausschalten
  setMagnetOff();  
  
  // im Suchmodus starten
  initFindGoal(false);

}
