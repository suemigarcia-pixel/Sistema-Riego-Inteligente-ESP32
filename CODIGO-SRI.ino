#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no esta habilitado en tu configuracion de ESP32.
#endif

BluetoothSerial SerialBT;

#define RXD2            16
#define TXD2            17
#define DE_RE           4
#define PIN_BOMBA_AGUA  26
#define PIN_SENSOR_AGUA 32

#define PIN_VALVULA_N   27
#define PIN_VALVULA_P   33
#define PIN_VALVULA_K   25
#define PIN_VALVULA_PH  13

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define USAR_SENSOR_NIVEL 1
#define USAR_VALVULAS     1

#define BOMBA_ON  HIGH
#define BOMBA_OFF LOW

#define VALV_ON   LOW
#define VALV_OFF  HIGH

#define RESCATE_PH_EN_AGUA 1
#define DEBUG_REGISTROS    1

#define BAUD_FIJO      0
#define DIR_FIJA       0
#define REINTENTOS     3
#define PAUSA_CONSULTA 150

const uint32_t BAUDS[] = {4800, 9600, 2400, 19200};
uint32_t baudActual = 0;
uint8_t  dirActual  = 0;

const float TEMP_UMBRAL_PICOR   = 22.0;
const float TEMP_UMBRAL_BOLTING = 7.0;

const int   HUMEDAD_SUELO_MIN   = 55;
const int   HUMEDAD_SUELO_MAX   = 80;

const float PH_OPTIMO_MIN       = 6.0;
const float PH_OPTIMO_MAX       = 7.2;

const uint16_t NITROGENO_MIN    = 40;
const uint16_t NITROGENO_MAX    = 120;

const uint16_t FOSFORO_MIN      = 15;
const uint16_t FOSFORO_MAX      = 50;

const uint16_t POTASIO_MIN      = 150;
const uint16_t POTASIO_MAX      = 300;

const unsigned long INTERVALO_LECTURA    = 2000;
const unsigned long INTERVALO_CAMBIO_LCD = 3000;
const unsigned long RIEGO_MAX_MS         = 120000UL;
const unsigned long PAUSA_RIEGO_MS       = 300000UL;
const uint8_t       FALLOS_PARA_ERROR    = 3;

struct Lectura {
  float    humSuelo, tempSuelo, ph;
  uint16_t ec, n, p, k;
};

Lectura lecturaGlobal;
bool datosValidos   = false;
bool filtroIniciado = false;

#define VENTANA_MEDIANA 5
uint16_t histEC[VENTANA_MEDIANA], histN[VENTANA_MEDIANA], histP[VENTANA_MEDIANA], histK[VENTANA_MEDIANA];
uint8_t idxFiltro = 0;
uint8_t muestrasAcumuladas = 0;

bool bombaAguaOn = false;
bool bloqueoAgua = false;

bool valvulaNOn  = false;
bool valvulaPOn  = false;
bool valvulaKOn  = false;
bool valvulaPhOn = false;

unsigned long tInicioRiego = 0, tBloqueoAgua = 0;
unsigned long tUltimaLectura = 0, tUltimoEscaneo = 0, tUltimoCambioLCD = 0;
uint8_t fallos = 0;

bool vistaActualLCD       = true;
bool solicitarLimpiezaLCD = true;

uint16_t crc16(const uint8_t *d, uint8_t n) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

uint16_t obtenerMediana(uint16_t *arr, uint8_t tam) {
  uint16_t copia[VENTANA_MEDIANA];
  for (uint8_t i = 0; i < tam; i++) copia[i] = arr[i];
  for (uint8_t i = 0; i < tam - 1; i++) {
    for (uint8_t j = i + 1; j < tam; j++) {
      if (copia[j] < copia[i]) {
        uint16_t tmp = copia[i];
        copia[i] = copia[j];
        copia[j] = tmp;
      }
    }
  }
  return copia[tam / 2];
}

bool consulta(uint8_t dir, uint16_t reg, uint8_t cant, uint16_t *val, uint16_t timeoutMs = 500) {
  uint8_t req[8] = {dir, 0x03, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), 0x00, cant};
  uint16_t crc = crc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  uint32_t esperaTxUs = (baudActual > 0) ? (12000000UL / baudActual) : 2500;

  while (Serial2.available()) Serial2.read();
  digitalWrite(DE_RE, HIGH);
  delayMicroseconds(500);
  Serial2.write(req, 8);
  Serial2.flush();
  delayMicroseconds(esperaTxUs);
  digitalWrite(DE_RE, LOW);

  const uint8_t esperado = 5 + 2 * cant;
  uint8_t resp[64];
  uint8_t n = 0;
  unsigned long t0 = millis();
  while (n < esperado && millis() - t0 < timeoutMs) {
    if (Serial2.available()) resp[n++] = Serial2.read();
  }

  if (n < esperado) return false;
  if (resp[0] != dir || resp[1] != 0x03 || resp[2] != 2 * cant) return false;
  uint16_t calc = crc16(resp, esperado - 2);
  uint16_t recv = resp[esperado - 2] | (resp[esperado - 1] << 8);
  if (calc != recv) return false;

  for (uint8_t i = 0; i < cant; i++)
    val[i] = (resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
  return true;
}

bool consultaReintento(uint8_t dir, uint16_t reg, uint8_t cant, uint16_t *val) {
  for (uint8_t i = 0; i < REINTENTOS; i++) {
    if (consulta(dir, reg, cant, val)) return true;
    delay(PAUSA_CONSULTA);
  }
  return false;
}

float decodificarPH(uint16_t rawPH) {
  if (rawPH == 0 || rawPH == 0xFFFF || rawPH == 0x7FFF) return 0.0;
  if (rawPH >= 300 && rawPH <= 1400) return rawPH / 100.0;
  if (rawPH >= 30 && rawPH <= 140)   return rawPH / 10.0;
  if (rawPH >= 3 && rawPH <= 14)     return (float)rawPH;
  return 0.0;
}

bool leerTodo(Lectura &L) {
  uint16_t v[7] = {0};

  if (!consultaReintento(dirActual, 0x0000, 7, v)) {
    for (uint8_t i = 0; i < 7; i++) {
      if (!consultaReintento(dirActual, i, 1, &v[i])) return false;
      delay(PAUSA_CONSULTA);
    }
  }

  float tempCruda = (int16_t)v[0] / 10.0;
  float humCruda  = v[1] / 10.0;

  if (tempCruda > 65.0 && humCruda < 50.0) {
    float tmp = tempCruda;
    tempCruda = humCruda;
    humCruda  = tmp;
  }

  if (humCruda > 100.0) humCruda = 100.0;
  if (humCruda < 0.0)   humCruda = 0.0;

  uint16_t ecCrudo = v[2];
  uint16_t nCrudo  = v[4];
  uint16_t pCrudo  = v[5];
  uint16_t kCrudo  = v[6];

  float phCrudo = decodificarPH(v[3]);

  if (phCrudo == 0.0) {
    const uint16_t dirsPH[] = {0x0008, 0x0009, 0x0016, 0x0007};
    uint16_t regExtra = 0;
    for (uint16_t d : dirsPH) {
      if (consulta(dirActual, d, 1, &regExtra, 120)) {
        float phPrueba = decodificarPH(regExtra);
        if (phPrueba >= 3.0 && phPrueba <= 11.0) {
          phCrudo = phPrueba;
          break;
        }
      }
    }
  }

  if (phCrudo == 0.0) {
    uint16_t npkAlt[3] = {0};
    if (consulta(dirActual, 0x001E, 3, npkAlt, 120)) {
      if ((npkAlt[0] + npkAlt[1] + npkAlt[2]) > 0 && npkAlt[0] < 2000) {
        float phEn6 = decodificarPH(v[6]);
        if (phEn6 >= 3.0 && phEn6 <= 11.0) {
          phCrudo = phEn6;
          nCrudo  = npkAlt[0];
          pCrudo  = npkAlt[1];
          kCrudo  = npkAlt[2];
        }
      }
    }
  }

  #if RESCATE_PH_EN_AGUA
    if (phCrudo == 0.0 && humCruda >= 15.0 && ecCrudo >= 30) {
      float offsetEC   = ((float)ecCrudo - 250.0) * 0.0008;
      float offsetTemp = (tempCruda - 25.0) * (-0.015);
      phCrudo = 7.0 + offsetEC + offsetTemp;
      if (phCrudo < 5.5) phCrudo = 5.5;
      if (phCrudo > 8.2) phCrudo = 8.2;
    }
  #endif

  #if DEBUG_REGISTROS
    Serial.printf("[RAW] T_R0:%u H_R1:%u EC_R2:%u pH_R3:%u N_R4:%u P_R5:%u K_R6:%u -> pH_Calc:%.2f\n",
                  v[0], v[1], v[2], v[3], v[4], v[5], v[6], phCrudo);
  #endif

  histEC[idxFiltro] = ecCrudo;
  histN[idxFiltro]  = nCrudo;
  histP[idxFiltro]  = pCrudo;
  histK[idxFiltro]  = kCrudo;
  idxFiltro = (idxFiltro + 1) % VENTANA_MEDIANA;
  if (muestrasAcumuladas < VENTANA_MEDIANA) muestrasAcumuladas++;

  uint16_t ecMed = obtenerMediana(histEC, muestrasAcumuladas);
  uint16_t nMed  = obtenerMediana(histN,  muestrasAcumuladas);
  uint16_t pMed  = obtenerMediana(histP,  muestrasAcumuladas);
  uint16_t kMed  = obtenerMediana(histK,  muestrasAcumuladas);

  if (!filtroIniciado) {
    L.humSuelo  = humCruda;
    L.tempSuelo = tempCruda;
    L.ph        = phCrudo;
    L.ec        = ecMed;
    L.n         = nMed;
    L.p         = pMed;
    L.k         = kMed;
    filtroIniciado = true;
  } else {
    const float ALPHA = 0.35;
    L.humSuelo  = (ALPHA * humCruda)  + ((1.0 - ALPHA) * lecturaGlobal.humSuelo);
    L.tempSuelo = (ALPHA * tempCruda) + ((1.0 - ALPHA) * lecturaGlobal.tempSuelo);
    if (phCrudo > 0.0) {
      L.ph = (lecturaGlobal.ph == 0.0) ? phCrudo : ((ALPHA * phCrudo) + ((1.0 - ALPHA) * lecturaGlobal.ph));
    } else {
      L.ph = lecturaGlobal.ph;
    }
    L.ec = (uint16_t)((ALPHA * ecMed) + ((1.0 - ALPHA) * lecturaGlobal.ec));
    L.n  = (uint16_t)((ALPHA * nMed)  + ((1.0 - ALPHA) * lecturaGlobal.n));
    L.p  = (uint16_t)((ALPHA * pMed)  + ((1.0 - ALPHA) * lecturaGlobal.p));
    L.k  = (uint16_t)((ALPHA * kMed)  + ((1.0 - ALPHA) * lecturaGlobal.k));
  }

  return true;
}

bool escanear() {
  uint16_t tmp;
  for (uint32_t baud : BAUDS) {
    baudActual = baud;
    Serial2.end();
    Serial2.begin(baud, SERIAL_8N1, RXD2, TXD2);
    Serial.printf("Probando %lu baudios...\n", (unsigned long)baud);
    for (uint16_t dir = 1; dir <= 10; dir++) {
      if (consulta(dir, 0x0000, 1, &tmp, 300)) {
        dirActual = dir;
        Serial.printf(">>> Sensor detectado a %lu baudios, direccion %u\n", (unsigned long)baud, dir);
        return true;
      }
    }
  }
  baudActual = 0;
  Serial.println("Error: No se encontro el sensor Modbus 7en1.");
  return false;
}

void gestionarLCD() {
  if (!datosValidos) return;

  unsigned long ahora = millis();

  if (ahora - tUltimoCambioLCD >= INTERVALO_CAMBIO_LCD) {
    tUltimoCambioLCD = ahora;
    vistaActualLCD = !vistaActualLCD;
    solicitarLimpiezaLCD = true;
  }

  if (solicitarLimpiezaLCD) {
    lcd.clear();
    solicitarLimpiezaLCD = false;
  }

  if (vistaActualLCD) {
    lcd.setCursor(0, 0);
    lcd.print("EC:"); lcd.print(lecturaGlobal.ec); lcd.print("   ");

    lcd.setCursor(9, 0);
    lcd.print("HS:"); lcd.print((int)lecturaGlobal.humSuelo); lcd.print("% ");

    lcd.setCursor(0, 1);
    lcd.print("TS:"); lcd.print(lecturaGlobal.tempSuelo, 1); lcd.print("C ");

    lcd.setCursor(9, 1);
    lcd.print("PH:"); lcd.print(lecturaGlobal.ph, 1); lcd.print(" ");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("N:"); lcd.print(lecturaGlobal.n); lcd.print("   ");

    lcd.setCursor(8, 0);
    lcd.print("P:"); lcd.print(lecturaGlobal.p); lcd.print("   ");

    lcd.setCursor(0, 1);
    lcd.print("K:"); lcd.print(lecturaGlobal.k); lcd.print("   ");

    lcd.setCursor(8, 1);
    lcd.print("mg/kg");
  }
}

void setBombaAgua(bool on) {
  bombaAguaOn = on;
  digitalWrite(PIN_BOMBA_AGUA, on ? BOMBA_ON : BOMBA_OFF);
  if (on) tInicioRiego = millis();
  Serial.println(on ? "[BOMBA] ENCENDIDA" : "[BOMBA] APAGADA");
}

void apagarValvulas() {
  valvulaNOn  = false; digitalWrite(PIN_VALVULA_N,  VALV_OFF);
  valvulaPOn  = false; digitalWrite(PIN_VALVULA_P,  VALV_OFF);
  valvulaKOn  = false; digitalWrite(PIN_VALVULA_K,  VALV_OFF);
  valvulaPhOn = false; digitalWrite(PIN_VALVULA_PH, VALV_OFF);
}

void controlarValvulas(const Lectura &L) {
  if (L.n < NITROGENO_MIN) valvulaNOn = true;
  else if (L.n > NITROGENO_MAX) valvulaNOn = false;
  digitalWrite(PIN_VALVULA_N, valvulaNOn ? VALV_ON : VALV_OFF);

  if (L.p < FOSFORO_MIN) valvulaPOn = true;
  else if (L.p > FOSFORO_MAX) valvulaPOn = false;
  digitalWrite(PIN_VALVULA_P, valvulaPOn ? VALV_ON : VALV_OFF);

  if (L.k < POTASIO_MIN) valvulaKOn = true;
  else if (L.k > POTASIO_MAX) valvulaKOn = false;
  digitalWrite(PIN_VALVULA_K, valvulaKOn ? VALV_ON : VALV_OFF);

  if (L.ph < PH_OPTIMO_MIN) valvulaPhOn = true;
  else if (L.ph > PH_OPTIMO_MAX) valvulaPhOn = false;
  digitalWrite(PIN_VALVULA_PH, valvulaPhOn ? VALV_ON : VALV_OFF);
}

bool nivelAguaBajo() {
  #if USAR_SENSOR_NIVEL
    return digitalRead(PIN_SENSOR_AGUA) == LOW;
  #else
    return false;
  #endif
}

void controlarSistema(const Lectura &L) {
  unsigned long ahora = millis();
  bool nivelBajo = nivelAguaBajo();

  if (nivelBajo) {
    if (bombaAguaOn) setBombaAgua(false);
    apagarValvulas();
  } else {
    if (bombaAguaOn) {
      if (L.humSuelo >= HUMEDAD_SUELO_MAX) {
        setBombaAgua(false);
      } else if (ahora - tInicioRiego >= RIEGO_MAX_MS) {
        setBombaAgua(false);
        bloqueoAgua  = true;
        tBloqueoAgua = ahora;
      }
    } else {
      if (bloqueoAgua && ahora - tBloqueoAgua >= PAUSA_RIEGO_MS) bloqueoAgua = false;
      if (!bloqueoAgua && L.humSuelo < HUMEDAD_SUELO_MIN) setBombaAgua(true);
    }

    #if USAR_VALVULAS
      controlarValvulas(L);
    #endif
  }

  Serial.printf("[CTRL] hum:%.1f nivelBajo:%d bomba:%d bloqueo:%d | valvulas N:%d P:%d K:%d pH:%d\n",
                L.humSuelo, nivelBajo, bombaAguaOn, bloqueoAgua,
                valvulaNOn, valvulaPOn, valvulaKOn, valvulaPhOn);
}

void enviarBluetooth(const Lectura &L) {
  char trama[90];
  int alertaAguaBaja = nivelAguaBajo() ? 1 : 0;
  int estadoBomba = bombaAguaOn ? 1 : 0;
  snprintf(trama, sizeof(trama), "%u,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u",
           L.ec, L.humSuelo, L.tempSuelo, L.ph, L.n, L.p, L.k, alertaAguaBaja, estadoBomba);

  Serial.println(trama);
  SerialBT.println(trama);
}

void falloSensor() {
  if (bombaAguaOn) setBombaAgua(false);
  apagarValvulas();

  lcd.setCursor(0, 0); lcd.print("Error de Sensor ");
  lcd.setCursor(0, 1); lcd.print("Verifique RS485 ");
  Serial.println("ERROR_RS485");
  SerialBT.println("ERR");
}

void iniciarSensor() {
  if (BAUD_FIJO && DIR_FIJA) {
    baudActual = BAUD_FIJO;
    dirActual  = DIR_FIJA;
    Serial2.begin(baudActual, SERIAL_8N1, RXD2, TXD2);
  } else if (!escanear()) {
    falloSensor();
    return;
  }
}

void setup() {
  Serial.begin(115200);

  digitalWrite(PIN_BOMBA_AGUA, BOMBA_OFF);
  pinMode(PIN_BOMBA_AGUA, OUTPUT);

  digitalWrite(PIN_VALVULA_N, VALV_OFF);
  pinMode(PIN_VALVULA_N, OUTPUT);
  digitalWrite(PIN_VALVULA_P, VALV_OFF);
  pinMode(PIN_VALVULA_P, OUTPUT);
  digitalWrite(PIN_VALVULA_K, VALV_OFF);
  pinMode(PIN_VALVULA_K, OUTPUT);
  digitalWrite(PIN_VALVULA_PH, VALV_OFF);
  pinMode(PIN_VALVULA_PH, OUTPUT);

  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);
  pinMode(PIN_SENSOR_AGUA, INPUT_PULLUP);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Sistema Raphanus");
  lcd.setCursor(0, 1); lcd.print("BT:ESP32_Raphan");

  SerialBT.begin("ESP32_Raphanus");
  delay(1500);

  iniciarSensor();
  tUltimoEscaneo = millis();
}

void loop() {
  unsigned long ahora = millis();

  gestionarLCD();

  if (baudActual == 0) {
    if (ahora - tUltimoEscaneo >= 10000) {
      iniciarSensor();
      tUltimoEscaneo = millis();
    }
    return;
  }

  if (ahora - tUltimaLectura < INTERVALO_LECTURA) return;
  tUltimaLectura = ahora;

  Lectura L = lecturaGlobal;
  if (leerTodo(L)) {
    fallos = 0;
    lecturaGlobal = L;
    datosValidos = true;

    controlarSistema(L);
    enviarBluetooth(L);
  } else {
    fallos++;
    if (fallos >= FALLOS_PARA_ERROR) {
      datosValidos = false;
      falloSensor();
    }
  }
}