/*
    SPI Motor Kommandos
*/

// erweiterte Abfrage-Makros
#define bitIsSet(src, bitno) bitRead(src, bitno)
#define bitIsClear(src, bitno) !bitRead(src, bitno)

// Richtungs-Codes
#define MOTOR_DirNorth  0
#define MOTOR_DirWest   1
#define MOTOR_DirSouth  2
#define MOTOR_DirEast   3
#define MOTOR_DirInvalid 4
// Kommando-Feld-Masken
#define MOTOR_DirMask   0x03   // Bits 0..1
#define MOTOR_CountMask 0x1C   // Bits 2..4
#define MOTOR_CmdMask   0xE0   // Bits 5..7
// Kommando-Codes 
#define MOTOR_CmdStatus 0      // Status-Abfrage
#define MOTOR_CmdDrag   1      // ziehen (mit Abfrage von Ziel und Zaun)
#define MOTOR_CmdMove   2      // fahren (ohne Ziel- und Zaun)
#define MOTOR_CmdCenter 3      // zentrieren
#define MOTOR_CmdStop   5      // Stop
// #define MOTOR_CmdReset  6      // zurücksetzen
#define MOTOR_CmdPosition 0xFF    // Positionsabfrage 

// Kommando erzeugen und Felder extrahieren
#define MOTOR_SPIcommand(dir, count, cmd)  ((dir) | (count) << 2 | (cmd) << 5)
#define MOTOR_CmdDirection(cmd) ((cmd) & MOTOR_DirMask)
#define MOTOR_CmdCount(cmd) ((cmd) & MOTOR_CountMask) >> 2
#define MOTOR_CmdCommand(cmd) ((cmd) & MOTOR_CmdMask) >> 5

// Position zerlegen
#define MOTOR_PosX(pos) ((pos) >> 3)
#define MOTOR_PosY(pos) ((pos) & 07)

// Status Bits
#define MOTOR_StatusMidBit      0       // auf Feldmitte
#define MOTOR_StatusGoalBit     1       // Zielkontakt
#define MOTOR_StatusFenceBit    2       // Zaunkontakt
#define MOTOR_StatusLimitBit    3       // Grenzschalter
// #define MOTOR_StatusWcolBit     4       // Write Collision Data Register
#define MOTOR_StatusOverrunBit  5       // Befehlspufferkonflikt       
#define MOTOR_StatusReadyBit    6       // Befehl beendet
#define MOTOR_StatusPositionBit 7       // 1: Position 0: Status           


