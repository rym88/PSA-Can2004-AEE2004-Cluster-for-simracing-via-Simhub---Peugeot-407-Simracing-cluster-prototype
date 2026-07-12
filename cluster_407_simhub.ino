/*
 * ============================================================================
 *  Peugeot 407 (AEE2004 / CAN2004) instrument cluster driver
 *  Cluster VDO p/n 9658138280 - CAN LOW SPEED (CONFORT) @ 125 kbps
 *
 *  Hardware:
 *    - Arduino UNO compatible (Ardushop Jade U1, ATmega328P, 16 MHz)
 *    - SparkFun CAN-Bus Shield (MCP2515 + MCP2551, quartz 16 MHz)
 *        CS  = D10,  INT = D2  (valorile standard ale shield-ului)
 *    - Cluster alimentat la +12V, GND comun cu Arduino
 *    - CANH / CANL shield -> pini CAN cluster
 *      (daca clusterul nu reactioneaza, pune o rezistenta de 120R
 *       intre CANH si CANL - clusterul nu are terminator intern)
 *
 *  Biblioteca necesara (Library Manager):
 *    "mcp_can" by coryjfowler  -> https://github.com/coryjfowler/MCP_CAN_lib
 *
 *  Surse pentru structura cadrelor (reverse engineering PSA):
 *    - https://github.com/prototux/PSA-RE          (buses/AEE2004.full/LS.CONF)
 *    - https://github.com/ludwig-v/arduino-psa-comfort-can-adapter
 *
 *  Cadre trimise catre cluster (CMB):
 *    0x036  ~100 ms  BSI status (tine clusterul treaz, iluminare)
 *    0x0B6  ~ 50 ms  Turatie (0.125 rpm/bit) + viteza (0.01 km/h/bit)
 *    0x0F6  ~200 ms  Stare motor, temp. apa, odometru, temp. ext.,
 *                    semnalizatoare, mars-arriere
 *    0x128  ~200 ms  Martori de bord (lumini, centuri, frana de mana...)
 *    0x168  ~200 ms  Alerte (baterie, presiune ulei, ABS, motor...)
 *    0x161  ~500 ms  Temp. ulei + nivel combustibil + nivel ulei
 *    0x1A1  la cerere: mesaj pop-up pe afisajul clusterului
 *
 *  Protocol serial (SimHub "Custom serial device", 115200 baud):
 *    Linie ASCII terminata cu '\n', perechi cheie:valoare separate de ';'
 *    Exemplu:
 *      rpm:4500;spd:132;wtr:92;oil:105;ful:55;lt:0;rt:1;lb:1;hb:0;msg:255\n
 *
 *    Chei acceptate (toate optionale, valorile lipsa raman neschimbate):
 *      rpm  : turatie motor [rot/min]
 *      spd  : viteza [km/h]
 *      wtr  : temperatura apa [grade C]
 *      oil  : temperatura ulei [grade C]
 *      ful  : nivel combustibil [0..100 %]
 *      ext  : temperatura exterioara [grade C]
 *      lt/rt: semnalizator stanga/dreapta (0/1)
 *      sl   : lumini pozitie (0/1)      lb: faza scurta   hb: faza lunga
 *      ffg  : proiectoare fata          rfg: ceata spate
 *      hbk  : frana de mana (0/1)       blt: centura (0/1)
 *      dor  : usa deschisa (0/1)        lfu: rezerva combustibil (0/1)
 *      stp  : martor STOP               srv: martor SERVICE
 *      esp  : ESP activ (clipire)       eso: ESP dezactivat
 *      abs  : defect ABS                bat: defect incarcare
 *      olp  : presiune ulei             eng: defect motor (MIL)
 *      rev  : mars-arriere (0/1)        ign: 0=contact pus, 1=motor pornit
 *      msg  : ID mesaj pop-up (0..255); 255 = inchide mesajul
 *
 *  Fara date seriale timp de 5 s -> intra automat in DEMO MODE
 *  (baleiaza turatia/viteza ca sa verifici ca totul functioneaza).
 * ============================================================================
 */

#include <SPI.h>
#include <mcp_can.h>

// ---------------------------------------------------------------- configurare
#define CAN_CS_PIN     10          // SparkFun CAN-Bus Shield: CS = D10
#define CAN_INT_PIN    2           // nefolosit aici (doar transmisie)
#define SERIAL_BAUD    115200
#define DEMO_TIMEOUT   5000UL      // ms fara date seriale -> demo mode

MCP_CAN CAN(CAN_CS_PIN);

// ------------------------------------------------------------ starea vehiculului
struct VehicleState {
  uint16_t rpm        = 0;       // rot/min
  uint16_t speedKmh   = 0;       // km/h
  int16_t  waterTemp  = 20;      // grade C
  int16_t  oilTemp    = 20;      // grade C
  uint8_t  fuelPct    = 50;      // %
  int16_t  extTemp    = 20;      // grade C

  // martori / lumini (0/1)
  uint8_t leftTurn = 0, rightTurn = 0;
  uint8_t sidelights = 0, lowBeam = 0, highBeam = 0;
  uint8_t frontFog = 0, rearFog = 0;
  uint8_t handbrake = 0, seatbelt = 0, doorOpen = 0, lowFuel = 0;
  uint8_t stopLamp = 0, serviceLamp = 0;
  uint8_t espActive = 0, espOff = 0;
  uint8_t absFault = 0, batteryFault = 0, oilPressure = 0, engineFault = 0;
  uint8_t reverse = 0;
  uint8_t engineRunning = 1;     // 0 = doar contact, 1 = motor pornit

  uint8_t  popupId   = 255;      // 255 = niciun mesaj
  uint8_t  popupSent = 1;
} veh;

// contori interni ceruti de cluster (odometru "viu" pentru consum/trip)
uint16_t tripDistance = 0;       // 0.1 m / bit  (cadrul 0x0B6)
uint8_t  fuelCounter  = 0;       // contor consum (cadrul 0x0B6)
uint32_t odometerKm10 = 1234567; // odometru afisat, in 0.1 km (123456.7 km)

// planificator simplu
unsigned long t036 = 0, t0B6 = 0, t0F6 = 0, t128 = 0, t168 = 0, t161 = 0;
unsigned long lastSerialRx = 0;
bool demoMode = true;

// ================================================================== SETUP
void setup() {
  Serial.begin(SERIAL_BAUD);

  while (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println(F("MCP2515 init FAILED - verifica shield-ul, reincerc..."));
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("MCP2515 OK - CAN 125kbps (PSA CONFORT / AEE2004)"));
  Serial.println(F("Astept date SimHub... (altfel demo mode in 5s)"));
}

// ================================================================== LOOP
void loop() {
  readSerial();

  // demo mode daca nu vin date de la SimHub
  if (millis() - lastSerialRx > DEMO_TIMEOUT) {
    demoMode = true;
    runDemo();
  }

  unsigned long now = millis();
  if (now - t036 >= 100) { t036 = now; send_036(); }
  if (now - t0B6 >=  50) { t0B6 = now; send_0B6(); }
  if (now - t0F6 >= 200) { t0F6 = now; send_0F6(); }
  if (now - t128 >= 200) { t128 = now; send_128(); }
  if (now - t168 >= 200) { t168 = now; send_168(); }
  if (now - t161 >= 500) { t161 = now; send_161(); }

  if (!veh.popupSent) { send_1A1(); veh.popupSent = 1; }
}

// ============================================================ CADRE CAN

// ---- 0x036 : BSI status / iluminare (tine clusterul activ) -----------------
// byte2 bit7 = mod economie (0 = normal)
// byte3 biti 3..0 = luminozitate (0x0F = max), bit5 = zi/noapte
// byte4 biti 2..0 = power management
void send_036() {
  byte d[8] = {0x0E, 0x00, 0x00, 0x0F, 0x01, 0x80, 0x00, 0xA0};
  CAN.sendMsgBuf(0x036, 0, 8, d);
}

// ---- 0x0B6 : turatie + viteza ----------------------------------------------
// bytes 0-1 : turatie, rezolutie 0.125 rpm/bit  (raw = rpm * 8)
// bytes 2-3 : viteza,  rezolutie 0.01 km/h/bit  (raw = kmh * 100)
// bytes 4-5 : distanta parcursa (0.1 m/bit) - contor crescator
// byte  6   : contor consum combustibil (pentru consum instantaneu)
// byte  7   : bit7 = informatie viteza valida
void send_0B6() {
  uint16_t rawRpm = veh.rpm * 8;
  uint16_t rawSpd = veh.speedKmh * 100;

  // incrementeaza distanta ~ la fiecare 50 ms in functie de viteza
  // v [km/h] -> v/3.6 [m/s] -> *0.05 s -> *10 (unitati de 0.1 m)
  tripDistance += (uint16_t)((veh.speedKmh * 10UL) / 72);
  if (veh.rpm > 500) fuelCounter += 1 + veh.rpm / 2000;

  byte d[8];
  d[0] = highByte(rawRpm);
  d[1] = lowByte(rawRpm);
  d[2] = highByte(rawSpd);
  d[3] = lowByte(rawSpd);
  d[4] = highByte(tripDistance);
  d[5] = lowByte(tripDistance);
  d[6] = fuelCounter;
  d[7] = 0x80;                       // viteza valida
  CAN.sendMsgBuf(0x0B6, 0, 8, d);
}

// ---- 0x0F6 : stare motor, temp apa, odometru, temp ext, semnalizare --------
// byte0: bit7-6 config (10=client), bit4-3 stare (01=contact),
//        bit2 alternator OK, bit1-0 GMP (10 = motor pornit)
// byte1: temperatura apa, offset -40  (raw = tempC + 40)
// bytes2-4: odometru, 0.1 km/bit, 24 biti
// byte5: temp ext (raw = (t+40)*2)   byte6: temp ext filtrata
// byte7: bit7 = mars-arriere, biti1-0 = semnalizatoare (01=dr,10=st,11=ambele)
void send_0F6() {
  byte d[8];
  d[0] = veh.engineRunning ? 0x8E : 0x88;      // 0x8E = motor pornit
  d[1] = (byte)constrain(veh.waterTemp + 40, 0, 255);
  d[2] = (odometerKm10 >> 16) & 0xFF;
  d[3] = (odometerKm10 >> 8) & 0xFF;
  d[4] = odometerKm10 & 0xFF;
  d[5] = (byte)constrain((veh.extTemp + 40) * 2, 0, 254);
  d[6] = d[5];
  d[7] = (veh.reverse ? 0x80 : 0x00)
       | (veh.leftTurn ? 0x02 : 0x00)
       | (veh.rightTurn ? 0x01 : 0x00);
  CAN.sendMsgBuf(0x0F6, 0, 8, d);
}

// ---- 0x128 : martori de bord (layout PSA-RE AEE2004) ------------------------
void send_128() {
  byte d[8] = {0};

  // byte0: b7 airbag pasager, b6 centura sofer, b5 frana de mana,
  //        b4 rezerva, b2 preincalzire, b1 centura pasager
  bitWrite(d[0], 6, veh.seatbelt);
  bitWrite(d[0], 5, veh.handbrake);
  bitWrite(d[0], 4, veh.lowFuel);

  // byte1: b7 service/mentenanta, b6 STOP, b4-b3 usi deschise
  bitWrite(d[1], 7, veh.serviceLamp);
  bitWrite(d[1], 6, veh.stopLamp);
  bitWrite(d[1], 4, veh.doorOpen);

  // byte2: b5 siguranta copii, b4 ESP dezactivat, b3 ESP activ (clipire),
  //        b1 avarii, b0 pregatit de pornire
  bitWrite(d[2], 4, veh.espOff);
  bitWrite(d[2], 3, veh.espActive);
  bitWrite(d[2], 1, (veh.leftTurn && veh.rightTurn) ? 1 : 0);  // avarii

  // byte3: b2-b1 pedala frana
  // byte4: lumini - b7 pozitie, b6 faza scurta, b5 faza lunga,
  //        b4 proiectoare fata, b3 ceata spate, b2 semn.dr, b1 semn.st, b0 DRL
  bitWrite(d[4], 7, veh.sidelights);
  bitWrite(d[4], 6, veh.lowBeam);
  bitWrite(d[4], 5, veh.highBeam);
  bitWrite(d[4], 4, veh.frontFog);
  bitWrite(d[4], 3, veh.rearFog);
  bitWrite(d[4], 2, veh.rightTurn);
  bitWrite(d[4], 1, veh.leftTurn);

  // byte5: b7 cluster ON
  bitWrite(d[5], 7, 1);

  CAN.sendMsgBuf(0x128, 0, 8, d);
}

// ---- 0x168 : alerte (baterie, presiune ulei, ABS, motor...) -----------------
void send_168() {
  byte d[8] = {0};
  // byte0: b7 alerta temp apa, b3 presiune ulei, b1 motor rece
  bitWrite(d[0], 7, veh.waterTemp >= 118 ? 1 : 0);
  bitWrite(d[0], 3, veh.oilPressure);
  // byte1: b2 zona rosie turometru (lvl1), b0 lvl2, b1 clipire rezerva
  bitWrite(d[1], 2, veh.rpm > 6000 ? 1 : 0);
  // byte3: b5 defect ABS, b4 defect ASR, b2 defect frane, b1 EOBD
  bitWrite(d[3], 5, veh.absFault);
  // byte4: b2 defect baterie, b1 defect alternator
  bitWrite(d[4], 2, veh.batteryFault);
  bitWrite(d[4], 1, veh.batteryFault);
  // byte5: b0 defect motor (MIL)
  bitWrite(d[5], 0, veh.engineFault);
  CAN.sendMsgBuf(0x168, 0, 8, d);
}

// ---- 0x161 : temperatura ulei + nivel combustibil (7 octeti!) ---------------
// byte2 = temp ulei + 40   byte3 = nivel combustibil (0..100)
// byte6 = nivel ulei (o valoare normala, ca sa nu apara alerta)
void send_161() {
  byte d[7] = {0};
  d[2] = (byte)constrain(veh.oilTemp + 40, 0, 255);
  d[3] = constrain(veh.fuelPct, 0, 100);
  d[6] = 0x1E;                      // nivel ulei OK
  CAN.sendMsgBuf(0x161, 0, 7, d);
}

// ---- 0x1A1 : mesaj pop-up pe afisajul clusterului ----------------------------
// byte0: bit7 = afiseaza mesaj, biti6-0 + byte1 = ID mesaj
// byte2: bit6 = destinatie CMB (cluster), biti3-0 = prioritate
// msg:255 pe serial -> inchide mesajul curent
void send_1A1() {
  byte d[8] = {0};
  if (veh.popupId == 255) {
    d[0] = 0x7F; d[1] = 0xFF;       // inchide pop-up
  } else {
    d[0] = 0x80;                    // mesaj nou
    d[1] = veh.popupId;             // ID mesaj (incearca 0,1,3,4,5,106,138...)
  }
  d[2] = 0x40 | 0x01;               // destinatie cluster + prioritate 1
  CAN.sendMsgBuf(0x1A1, 0, 8, d);
}

// ============================================================ PARSER SERIAL
// Linii "cheie:valoare;cheie:valoare;...\n" - vezi antetul fisierului.
char tokenBuf[16];
uint8_t tokenLen = 0;

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == ';' || c == '\n' || c == '\r') {
      if (tokenLen > 0) { tokenBuf[tokenLen] = 0; parseToken(tokenBuf); }
      tokenLen = 0;
      if (c == '\n') { lastSerialRx = millis(); demoMode = false; }
    } else if (tokenLen < sizeof(tokenBuf) - 1) {
      tokenBuf[tokenLen++] = c;
    }
  }
}

void parseToken(char *tok) {
  char *sep = strchr(tok, ':');
  if (!sep) return;
  *sep = 0;
  int val = atoi(sep + 1);

  if      (!strcmp(tok, "rpm")) veh.rpm       = constrain(val, 0, 8000);
  else if (!strcmp(tok, "spd")) veh.speedKmh  = constrain(val, 0, 300);
  else if (!strcmp(tok, "wtr")) veh.waterTemp = constrain(val, -40, 214);
  else if (!strcmp(tok, "oil")) veh.oilTemp   = constrain(val, -40, 214);
  else if (!strcmp(tok, "ful")) veh.fuelPct   = constrain(val, 0, 100);
  else if (!strcmp(tok, "ext")) veh.extTemp   = constrain(val, -40, 85);
  else if (!strcmp(tok, "lt"))  veh.leftTurn  = val ? 1 : 0;
  else if (!strcmp(tok, "rt"))  veh.rightTurn = val ? 1 : 0;
  else if (!strcmp(tok, "sl"))  veh.sidelights = val ? 1 : 0;
  else if (!strcmp(tok, "lb"))  veh.lowBeam   = val ? 1 : 0;
  else if (!strcmp(tok, "hb"))  veh.highBeam  = val ? 1 : 0;
  else if (!strcmp(tok, "ffg")) veh.frontFog  = val ? 1 : 0;
  else if (!strcmp(tok, "rfg")) veh.rearFog   = val ? 1 : 0;
  else if (!strcmp(tok, "hbk")) veh.handbrake = val ? 1 : 0;
  else if (!strcmp(tok, "blt")) veh.seatbelt  = val ? 1 : 0;
  else if (!strcmp(tok, "dor")) veh.doorOpen  = val ? 1 : 0;
  else if (!strcmp(tok, "lfu")) veh.lowFuel   = val ? 1 : 0;
  else if (!strcmp(tok, "stp")) veh.stopLamp  = val ? 1 : 0;
  else if (!strcmp(tok, "srv")) veh.serviceLamp = val ? 1 : 0;
  else if (!strcmp(tok, "esp")) veh.espActive = val ? 1 : 0;
  else if (!strcmp(tok, "eso")) veh.espOff    = val ? 1 : 0;
  else if (!strcmp(tok, "abs")) veh.absFault  = val ? 1 : 0;
  else if (!strcmp(tok, "bat")) veh.batteryFault = val ? 1 : 0;
  else if (!strcmp(tok, "olp")) veh.oilPressure  = val ? 1 : 0;
  else if (!strcmp(tok, "eng")) veh.engineFault  = val ? 1 : 0;
  else if (!strcmp(tok, "rev")) veh.reverse   = val ? 1 : 0;
  else if (!strcmp(tok, "ign")) veh.engineRunning = val ? 1 : 0;
  else if (!strcmp(tok, "msg")) {
    uint8_t id = (uint8_t)constrain(val, 0, 255);
    if (id != veh.popupId) { veh.popupId = id; veh.popupSent = 0; }
  }
}

// ============================================================ DEMO MODE
// Baleiaza turatia si viteza + aprinde pe rand cateva martori,
// ca sa verifici cablajul fara SimHub.
void runDemo() {
  static unsigned long tDemo = 0;
  static uint8_t phase = 0;
  if (millis() - tDemo < 40) return;
  tDemo = millis();

  static int dir = 60;
  veh.rpm += dir;
  if (veh.rpm >= 6000 || veh.rpm == 0) dir = -dir;
  veh.speedKmh = veh.rpm / 30;
  veh.waterTemp = 90;
  veh.oilTemp   = 100;
  veh.fuelPct   = 65;

  // roteste martorii la ~1.5 s
  static unsigned long tLamp = 0;
  if (millis() - tLamp > 1500) {
    tLamp = millis();
    phase = (phase + 1) % 6;
    veh.leftTurn   = (phase == 0);
    veh.rightTurn  = (phase == 1);
    veh.lowBeam    = (phase == 2);
    veh.highBeam   = (phase == 3);
    veh.handbrake  = (phase == 4);
    veh.seatbelt   = (phase == 5);
  }
}
