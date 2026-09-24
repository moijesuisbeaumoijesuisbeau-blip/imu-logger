// =================================================
// XIAO nRF52840 Sense (Plus)
// LSM6DS3TR-C FIFO — ACCEL 6664 Hz / GYRO 1666 Hz
// FLASH QSPI 2 Mo embarquee + BLE (Nordic UART)
//
// Version simplifiee : plus de GPS, plus de SD/carte
// d'extension. Uniquement l'acquisition IMU, le stockage
// tampon sur la flash embarquee, et l'envoi Bluetooth.
//
// CORRECTIFS APPLIQUES (matériel XIAO Sense) :
//  - Le capteur embarque est sur Wire1 (bus I2C interne
//    separe), pas sur Wire (bus externe). Toutes les
//    lectures FIFO manuelles utilisent donc Wire1.
//  - Frequences d'echantillonnage exactes reconnues par
//    la librairie : 6664 Hz (accel) et 1666 Hz (gyro),
//    pas 6660/1660 comme on pourrait l'arrondir.
//  - myIMU.settings.commMode = 1 ajoute (present dans
//    l'exemple officiel qui fonctionne, absent avant).
//
// A VALIDER : la vitesse I2C ci-dessous (1 MHz) n'a
// jamais ete testee sur Wire1 specifiquement (les tests
// precedents a 1 MHz utilisaient Wire par erreur). Testez
// d'abord avec le sketch de diagnostic minimal ; si ca
// echoue, redescendre a Wire1.setClock(400000).
// =================================================


#include <Adafruit_TinyUSB.h>

#include <SPI.h>
#include <Adafruit_SPIFlash.h>

#include <bluefruit.h>

#include <Wire.h>
#include "LSM6DS3.h"


// =================================================
// FLASH QSPI EMBARQUEE (2 Mo, puce P25Q16H)
// =================================================

Adafruit_FlashTransport_QSPI flashTransport;

static SPIFlash_Device_t const P25Q16H_MANUEL = {
  .total_size = (1UL << 21), // 2 MiB
  .start_up_time_us = 10000,
  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,
  .max_clock_speed_mhz = 55,
  .quad_enable_bit_mask = 0x02,
  .has_sector_protection = true,
  .supports_fast_read = true,
  .supports_qspi = true,
  .supports_qspi_writes = true,
  .write_status_register_split = true,
  .single_status_byte = false,
  .is_fram = false,
};

static SPIFlash_Device_t const APPAREILS_POSSIBLES[] = { P25Q16H_MANUEL };

Adafruit_SPIFlash flash(&flashTransport);


// =================================================
// BLE — NORDIC UART SERVICE (BLEUart)
// =================================================

BLEUart bleuart;


// =================================================
// IMU
// =================================================

#define IMU_ADDR 0x6A

LSM6DS3 myIMU(I2C_MODE, IMU_ADDR);


// =================================================
// ACQUISITION
// =================================================

uint32_t debutAcquisition = 0;

bool acquisition = false;


// =================================================
// TYPE D'ENREGISTREMENT
// =================================================

#define TYPE_ACCEL 0x01
#define TYPE_GYRO  0x02


// =================================================
// STRUCTURE D'UN ENREGISTREMENT (12 octets)
// =================================================

struct __attribute__((packed)) Enregistrement
{
  uint32_t temps;
  uint8_t type;
  uint8_t reserve;
  int16_t x;
  int16_t y;
  int16_t z;
};


// =================================================
// ENTETE MINIMALE SUR FLASH (adresse 0)
// =================================================

struct __attribute__((packed)) EnteteFlash
{
  char magic[4]; // "IMU3"
  uint8_t version;
};

#define HEADER_SIZE sizeof(EnteteFlash)


// =================================================
// ENTETE ENVOYEE EN BLUETOOTH AU DEBUT DE CHAQUE ENVOI
// =================================================

struct __attribute__((packed)) EnteteTransfert
{
  char magic[4]; // "IMUC"
  uint8_t version;

  uint32_t dureeMicros;
  uint32_t totalAccel;
  uint32_t totalGyro;
  uint32_t erreursGyro;

  uint32_t indexAccelDebut;
  uint32_t indexGyroDebut;
};


// =================================================
// PROTOTYPES
// =================================================

void initialiserIMU();
void demarrerAcquisition();
void remplirBuffer();
void ecrireBufferFlash();
void configurerBLE();
void demarrerEnvoi();
void etapeEnvoi();
void erreurFatale(uint8_t led);

// Clignote une LED en boucle (ROUGE = flash, BLEUE = IMU).
// Le BLE continue d'emettre en arriere-plan.
void erreurFatale(uint8_t led)
{
  while (1)
  {
    digitalWrite(led, LOW);
    delay(200);
    digitalWrite(led, HIGH);
    delay(200);
  }
}


// =================================================
// SETUP
// =================================================

void setup()
{
  Serial.begin(115200);

  uint32_t depart = millis();
  while (!Serial && (millis() - depart) < 3000) delay(100);
  delay(200);

  Serial.println();
  Serial.println("=================================");
  Serial.println("LOGGER XIAO nRF52840 — FLASH + BLE (sans GPS)");
  Serial.println("=================================");

  // =================================================
  // FLASH QSPI
  // =================================================

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  // BLE demarre EN PREMIER : la carte reste visible meme si
  // la flash ou l'IMU echouent ensuite.
  configurerBLE();
  digitalWrite(LED_BLUE, LOW);   // bleu fixe = BLE demarre
  delay(500);
  digitalWrite(LED_BLUE, HIGH);

  Serial.println("Initialisation flash QSPI...");

  if (!flash.begin(APPAREILS_POSSIBLES, 1))
  {
    Serial.println("ERREUR FLASH");
    erreurFatale(LED_RED);
  }

  Serial.print("Flash OK, taille detectee : ");
  Serial.print(flash.size() / 1024);
  Serial.println(" Ko");

  Serial.println("Effacement complet de la flash (peut prendre quelques dizaines de secondes)...");

  uint32_t t0 = millis();

  if (!flash.eraseChip())
  {
    Serial.println("ERREUR effacement flash");
    erreurFatale(LED_RED);
  }

  flash.waitUntilReady();

  Serial.print("Flash effacee en ");
  Serial.print((millis() - t0) / 1000);
  Serial.println(" s");

  // =================================================
  // I2C (Wire1 : bus interne de la XIAO Sense, ou est
  // cable le capteur embarque)
  // =================================================

  Wire1.begin();
  Wire1.setClock(1000000); // a valider avec le diagnostic ; sinon 400000

  // =================================================
  // IMU
  // =================================================

  initialiserIMU();

  // =================================================
  // ENTETE MINIMALE SUR FLASH
  // =================================================

  EnteteFlash entete;
  memcpy(entete.magic, "IMU3", 4);
  entete.version = 1;

  flash.writeBuffer(0, (uint8_t*)&entete, sizeof(entete));

  Serial.println("Entete ecrite sur flash");

  // =================================================
  // BLE
  // =================================================

  // (BLE deja demarre plus haut)

  // =================================================
  // DEMARRAGE ACQUISITION (immediat, pas de GPS a attendre)
  // =================================================

  debutAcquisition = micros();
  acquisition = true;

  demarrerAcquisition();
  digitalWrite(LED_GREEN, LOW);  // vert fixe = tout OK, acquisition en cours
}


// =================================================
// CONFIGURATION BLE
// =================================================

void configurerBLE()
{
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XIAO-IMU-LOGGER");

  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.start(0);

  Serial.println("BLE pret (nom : XIAO-IMU-LOGGER), en attente de connexion...");
}
// =================================================
// BUFFERS RAM (double buffer)
// =================================================

#define TAILLE_BUFFER 6000

#define SEUIL_ECRITURE ((TAILLE_BUFFER * 3) / 4)

Enregistrement bufferA[TAILLE_BUFFER];
Enregistrement bufferB[TAILLE_BUFFER];

Enregistrement *bufferEcriture = bufferA;
Enregistrement *bufferSD = bufferB;

uint16_t indexBuffer = 0;

uint16_t elementsSD = 0;

bool blocPret = false;


// =================================================
// CAPACITE FLASH UTILE ET SEUIL D'ENVOI (3/4)
// =================================================

#define FLASH_TOTALE (2UL * 1024UL * 1024UL)
#define FLASH_UTILE (FLASH_TOTALE - HEADER_SIZE)
#define SEUIL_FLASH ((FLASH_UTILE * 3UL) / 4UL)

uint32_t curseurEcriture = 0;
uint32_t curseurEnvoye = 0;

bool envoiDejaDeclencheAuto = false;


// =================================================
// COMPTEURS
// =================================================

uint32_t compteurAccel = 0;
uint32_t compteurGyro = 0;
uint32_t compteurTotal = 0;
uint32_t compteurErreursGyro = 0;

uint32_t accelEnvoyes = 0;
uint32_t gyroEnvoyes = 0;


// =================================================
// NOMBRE DE MOTS DISPONIBLES DANS LE FIFO (Wire1, rafale)
// =================================================

uint16_t lireNombreMotsFIFO()
{
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(0x3A);

  if (Wire1.endTransmission(true) != 0)
  {
    return 0;
  }

  if (Wire1.requestFrom(IMU_ADDR, (uint8_t)2) != 2)
  {
    return 0;
  }

  uint8_t status1 = Wire1.read();
  uint8_t status2 = Wire1.read();

  uint16_t status = ((uint16_t)status2 << 8) | status1;

  return status & 0x0FFF;
}


// =================================================
// LECTURE PATTERN + MOT FIFO (Wire1, rafale 4 octets)
// =================================================

bool lireMotEtPatternFIFO(uint16_t &pattern, int16_t &valeur)
{
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(0x3C);

  if (Wire1.endTransmission(true) != 0)
  {
    return false;
  }

  if (Wire1.requestFrom(IMU_ADDR, (uint8_t)4) != 4)
  {
    return false;
  }

  uint8_t patternL = Wire1.read();
  uint8_t patternH = Wire1.read();
  uint8_t donneeL  = Wire1.read();
  uint8_t donneeH  = Wire1.read();

  pattern = patternL | ((uint16_t)(patternH & 0x03) << 8);

  valeur = (int16_t)((uint16_t)donneeL | ((uint16_t)donneeH << 8));

  return true;
}


// =================================================
// AJOUT ACCEL / GYRO
// =================================================

void ajouterAccel(int16_t x, int16_t y, int16_t z)
{
  Enregistrement &e = bufferEcriture[indexBuffer];

  e.temps = (uint32_t)(((uint64_t)compteurAccel * 1000000ULL) / 6664ULL);

  e.type = TYPE_ACCEL;
  e.reserve = 0;

  e.x = x;
  e.y = y;
  e.z = z;

  indexBuffer++;
  compteurAccel++;
  compteurTotal++;
}

void ajouterGyro(int16_t x, int16_t y, int16_t z)
{
  Enregistrement &e = bufferEcriture[indexBuffer];

  e.temps = (uint32_t)(((uint64_t)compteurGyro * 1000000ULL) / 1666ULL);

  e.type = TYPE_GYRO;
  e.reserve = 0;

  e.x = x;
  e.y = y;
  e.z = z;

  indexBuffer++;
  compteurGyro++;
  compteurTotal++;
}


// =================================================
// CONFIGURATION IMU
// =================================================

void initialiserIMU()
{
  Serial.println("Initialisation IMU...");

  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = 2000;
  myIMU.settings.gyroSampleRate = 1666; // valeur exacte reconnue (pas 1660)
  myIMU.settings.gyroBandWidth = 200;
  myIMU.settings.gyroFifoEnabled = 1;
  myIMU.settings.gyroFifoDecimation = 1;

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 16;
  myIMU.settings.accelSampleRate = 6664; // valeur exacte reconnue (pas 6660)
  myIMU.settings.accelBandWidth = 200;
  myIMU.settings.accelFifoEnabled = 1;
  myIMU.settings.accelFifoDecimation = 1;

  myIMU.settings.timestampEnabled = 0;
  myIMU.settings.timestampFifoEnabled = 0;
  myIMU.settings.timestampResolution = 0;

  myIMU.settings.fifoThreshold = 2000;
  myIMU.settings.fifoSampleRate = 6600;
  myIMU.settings.fifoModeWord = 6;

  myIMU.settings.commMode = 1; // present dans l'exemple officiel, ajoute ici

  if (myIMU.begin() != 0)
  {
    Serial.println("ERREUR IMU");
    erreurFatale(LED_BLUE);
  }

  Serial.println("IMU OK");
}


// =================================================
// DEMARRAGE FIFO
// =================================================

void demarrerAcquisition()
{
  Serial.println("Demarrage FIFO...");

  myIMU.fifoBegin();
  myIMU.fifoClear();

  compteurAccel = 0;
  compteurGyro = 0;
  compteurTotal = 0;
  compteurErreursGyro = 0;
  indexBuffer = 0;

  debutAcquisition = micros();

  Serial.println("FIFO initialisee");
  Serial.println("ACCEL : 6664 Hz");
  Serial.println("GYRO  : 1666 Hz");
}


// =================================================
// LECTURE D'UN BLOC FIFO
// =================================================

int16_t etatGx = 0, etatGy = 0, etatGz = 0;
uint8_t etapeGyro = 0;

int16_t etatAx = 0, etatAy = 0;
uint8_t etapeAccel = 0;

#define LIMITE_MOTS_PAR_APPEL 1500

void remplirBuffer()
{
  uint16_t motsDisponibles = lireNombreMotsFIFO();

  if (motsDisponibles == 0)
  {
    return;
  }

  uint16_t aLire = motsDisponibles;

  if (aLire > LIMITE_MOTS_PAR_APPEL)
  {
    aLire = LIMITE_MOTS_PAR_APPEL;
  }

  for (uint16_t i = 0; i < aLire; i++)
  {
    uint16_t pattern;
    int16_t valeur;

    if (!lireMotEtPatternFIFO(pattern, valeur))
    {
      break;
    }

    if (pattern <= 2)
    {
      if (pattern == 0)
      {
        etatGx = valeur;
        etapeGyro = 1;
      }
      else if (pattern == 1 && etapeGyro == 1)
      {
        etatGy = valeur;
        etapeGyro = 2;
      }
      else if (pattern == 2 && etapeGyro == 2)
      {
        etatGz = valeur;
        ajouterGyro(etatGx, etatGy, etatGz);
        etapeGyro = 0;
      }
      else
      {
        compteurErreursGyro++;
        etapeGyro = (pattern == 0) ? 1 : 0;
      }
    }
    else
    {
      if (etapeAccel == 0)
      {
        etatAx = valeur;
        etapeAccel = 1;
      }
      else if (etapeAccel == 1)
      {
        etatAy = valeur;
        etapeAccel = 2;
      }
      else
      {
        ajouterAccel(etatAx, etatAy, valeur);
        etapeAccel = 0;
      }
    }

    if (indexBuffer >= SEUIL_ECRITURE)
    {
      Enregistrement *temp = bufferEcriture;
      bufferEcriture = bufferSD;
      bufferSD = temp;

      elementsSD = indexBuffer;
      indexBuffer = 0;

      blocPret = true;

      return;
    }
  }
}


// =================================================
// ECRITURE BUFFER SUR FLASH
// =================================================

void ecrireBufferFlash()
{
  size_t octets = sizeof(Enregistrement) * elementsSD;

  uint32_t restant = FLASH_UTILE - curseurEcriture;

  if (octets > restant)
  {
    octets = (restant / sizeof(Enregistrement)) * sizeof(Enregistrement);
  }

  if (octets > 0)
  {
    flash.writeBuffer(HEADER_SIZE + curseurEcriture, (uint8_t*)bufferSD, octets);
    curseurEcriture += octets;
  }

  static uint32_t blocs = 0;
  blocs++;

  Serial.print("Bloc flash : ");
  Serial.print(blocs);
  Serial.print(" | ");
  Serial.print((curseurEcriture * 100UL) / FLASH_UTILE);
  Serial.println("% flash utilisee");
}
// =================================================
// COMPRESSION DELTA + ENVOI BLE (Nordic UART)
// =================================================

struct EtatCompression
{
  int16_t refAccel[3] = {0, 0, 0};
  int16_t refGyro[3]  = {0, 0, 0};
};

EtatCompression etatCompressionEnvoi;

bool envoiEnCours = false;

uint32_t curseurLecture = 0;
uint32_t limiteEnvoi = 0;

#define RECORDS_PAR_PASSE 50
#define TAILLE_TAMPON_ENVOI 512

uint8_t tamponEnvoi[TAILLE_TAMPON_ENVOI];
uint16_t positionTampon = 0;


size_t encoderEnregistrement(const Enregistrement &e, EtatCompression &etat, uint8_t *sortie)
{
  int16_t *ref = (e.type == TYPE_GYRO) ? etat.refGyro : etat.refAccel;

  uint8_t tag = (e.type == TYPE_GYRO) ? 0x80 : 0x00;

  int16_t valeurs[3] = { e.x, e.y, e.z };

  uint8_t tampon[6];
  uint8_t pos = 0;

  for (uint8_t axe = 0; axe < 3; axe++)
  {
    int32_t delta = (int32_t)valeurs[axe] - (int32_t)ref[axe];

    if (delta >= -128 && delta <= 127)
    {
      tampon[pos++] = (uint8_t)(int8_t)delta;
    }
    else
    {
      tag |= (0x40 >> axe);
      tampon[pos++] = (uint8_t)(valeurs[axe] & 0xFF);
      tampon[pos++] = (uint8_t)((valeurs[axe] >> 8) & 0xFF);
    }

    ref[axe] = valeurs[axe];
  }

  sortie[0] = tag;
  memcpy(&sortie[1], tampon, pos);

  return 1 + pos;
}


void demarrerEnvoi()
{
  if (envoiEnCours)
  {
    Serial.println("Envoi deja en cours.");
    return;
  }

  if (curseurEnvoye >= curseurEcriture)
  {
    Serial.println("Rien de nouveau a envoyer.");
    return;
  }

  if (!Bluefruit.connected())
  {
    Serial.println("Aucun appareil BLE connecte : envoi impossible pour l'instant.");
    return;
  }

  limiteEnvoi = curseurEcriture;
  curseurLecture = curseurEnvoye;

  etatCompressionEnvoi = EtatCompression();
  positionTampon = 0;

  EnteteTransfert entete;
  memcpy(entete.magic, "IMUC", 4);
  entete.version = 1;

  entete.dureeMicros = micros() - debutAcquisition;
  entete.erreursGyro = compteurErreursGyro;

  entete.indexAccelDebut = accelEnvoyes;
  entete.indexGyroDebut = gyroEnvoyes;

  entete.totalAccel = compteurAccel - accelEnvoyes;
  entete.totalGyro = compteurGyro - gyroEnvoyes;

  bleuart.write((uint8_t*)&entete, sizeof(entete));

  envoiEnCours = true;

  Serial.print("Debut envoi BLE : ");
  Serial.print(limiteEnvoi - curseurLecture);
  Serial.println(" octets bruts a compresser et transmettre.");
}


void etapeEnvoi()
{
  uint8_t tamponRecord[8];

  for (uint16_t i = 0; i < RECORDS_PAR_PASSE && curseurLecture < limiteEnvoi; i++)
  {
    Enregistrement e;

    flash.readBuffer(HEADER_SIZE + curseurLecture, (uint8_t*)&e, sizeof(Enregistrement));

    curseurLecture += sizeof(Enregistrement);

    size_t taille = encoderEnregistrement(e, etatCompressionEnvoi, tamponRecord);

    if (positionTampon + taille > TAILLE_TAMPON_ENVOI)
    {
      bleuart.write(tamponEnvoi, positionTampon);
      positionTampon = 0;
    }

    memcpy(&tamponEnvoi[positionTampon], tamponRecord, taille);
    positionTampon += taille;

    if (e.type == TYPE_ACCEL)
    {
      accelEnvoyes++;
    }
    else
    {
      gyroEnvoyes++;
    }
  }

  if (curseurLecture >= limiteEnvoi)
  {
    if (positionTampon > 0)
    {
      bleuart.write(tamponEnvoi, positionTampon);
      positionTampon = 0;
    }

    curseurEnvoye = limiteEnvoi;
    envoiEnCours = false;

    Serial.println("Envoi BLE termine.");
  }
}


// =================================================
// STATISTIQUES
// =================================================

void afficherStatistiques()
{
  static uint32_t derniereAffichage = 0;
  static uint32_t dernierAccel = 0;
  static uint32_t dernierGyro = 0;

  if (millis() - derniereAffichage < 1000)
  {
    return;
  }

  derniereAffichage = millis();

  uint32_t accel = compteurAccel - dernierAccel;
  uint32_t gyro = compteurGyro - dernierGyro;

  dernierAccel = compteurAccel;
  dernierGyro = compteurGyro;

  Serial.print("ACCEL/s = ");
  Serial.print(accel);
  Serial.print(" | GYRO/s = ");
  Serial.print(gyro);
  Serial.print(" | Erreurs gyro = ");
  Serial.print(compteurErreursGyro);
  Serial.print(" | FIFO mots = ");
  Serial.print(lireNombreMotsFIFO());
  Serial.print(" | Flash = ");
  Serial.print((curseurEcriture * 100UL) / FLASH_UTILE);
  Serial.print("% | BLE = ");
  Serial.println(Bluefruit.connected() ? "connecte" : "en attente");
}


// =================================================
// LOOP
// =================================================

void loop()
{
  if (acquisition)
  {
    remplirBuffer();

    if (blocPret)
    {
      blocPret = false;
      ecrireBufferFlash();
    }

    afficherStatistiques();

    if (!envoiDejaDeclencheAuto && curseurEcriture >= SEUIL_FLASH)
    {
      envoiDejaDeclencheAuto = true;
      demarrerEnvoi();
    }

    if (curseurEcriture >= FLASH_UTILE)
    {
      acquisition = false;
      Serial.println("Flash pleine : acquisition arretee.");
    }
  }

  if (Serial.available())
  {
    char c = Serial.read();

    if (c == 's' || c == 'S')
    {
      demarrerEnvoi();
    }
  }

  if (envoiEnCours)
  {
    etapeEnvoi();
  }
}
