#include <Arduino.h>
#include <DueTimer.h>
#include <stdlib.h>
#include <string.h>

void envoyerOctet(byte donnee);

namespace {

constexpr uint32_t DEFAULT_BIT_RATE = 1000;
constexpr uint32_t MIN_BIT_RATE = 1000;
constexpr uint32_t MAX_BIT_RATE = 50000;
constexpr uint16_t RING_SIZE = 1024;
constexpr uint16_t RING_MASK = RING_SIZE - 1;
constexpr uint16_t START_THRESHOLD = 64;
constexpr uint16_t END_OF_TRANSFER = 0x100;
constexpr uint16_t PWM_CHANNEL = 7;
constexpr uint16_t PWM_PERIOD = 840;
constexpr uint16_t PWM_HALF_PERIOD = PWM_PERIOD / 2;
constexpr uint32_t SERIAL_READ_TIMEOUT_MS = 30000;
constexpr uint16_t MAX_IMAGE_DIMENSION = 320;
constexpr uint8_t SOT = 0xF0;
constexpr uint8_t EOT = 0x0F;

volatile uint16_t txRing[RING_SIZE];
volatile uint16_t txHead = 0;
volatile uint16_t txTail = 0;
volatile bool transmissionArmee = false;
volatile bool transmissionTerminee = false;
volatile int8_t bitIndex = -1;
volatile uint8_t octetActuel = 0;

char commande[40];
uint8_t longueurCommande = 0;
bool commandeTropLongue = false;

void appliquerPorteuse(bool active) {
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CDTYUPD =
      active ? PWM_HALF_PERIOD : 0;
}

void transmettreBit() {
  if (!transmissionArmee) {
    return;
  }

  if (bitIndex < 0) {
    if (txTail == txHead) {
      appliquerPorteuse(false);
      return;
    }

    const uint16_t element = txRing[txTail];
    txTail = (txTail + 1) & RING_MASK;

    if (element == END_OF_TRANSFER) {
      appliquerPorteuse(false);
      transmissionArmee = false;
      transmissionTerminee = true;
      return;
    }

    octetActuel = static_cast<uint8_t>(element);
    bitIndex = 0;
  }

  bool bit;
  if (bitIndex == 0) {
    bit = true;
  } else if (bitIndex <= 8) {
    bit = (octetActuel & (1U << (bitIndex - 1))) != 0;
  } else {
    bit = false;
  }

  appliquerPorteuse(bit);
  ++bitIndex;
  if (bitIndex == 10) {
    bitIndex = -1;
  }
}

uint16_t nombreOctetsEnFile() {
  return (txHead - txTail) & RING_MASK;
}

void armerSiPret(bool forcer = false) {
  if (forcer || nombreOctetsEnFile() >= START_THRESHOLD) {
    transmissionArmee = true;
  }
}

void mettreEnFile(uint16_t element, bool forcerDemarrage = false) {
  const uint16_t prochain = (txHead + 1) & RING_MASK;
  while (prochain == txTail) {
    yield();
  }

  txRing[txHead] = element;
  txHead = prochain;
  armerSiPret(forcerDemarrage);
}

bool lireOctetUSB(uint8_t &valeur) {
  const uint32_t debut = millis();
  while (Serial.available() == 0) {
    if (millis() - debut >= SERIAL_READ_TIMEOUT_MS) {
      return false;
    }
    yield();
  }

  const int recu = Serial.read();
  if (recu < 0) {
    return false;
  }
  valeur = static_cast<uint8_t>(recu);
  return true;
}

bool lireUInt16LEUSB(uint16_t &valeur) {
  uint8_t lsb;
  uint8_t msb;
  if (!lireOctetUSB(lsb) || !lireOctetUSB(msb)) {
    return false;
  }
  valeur = static_cast<uint16_t>(lsb) |
           (static_cast<uint16_t>(msb) << 8);
  return true;
}

bool transmettreTrame(const char *mode) {
  uint8_t expectedType;
  if (strcmp(mode, "TXT") == 0) {
    expectedType = 0x8F;
  } else if (strcmp(mode, "IMGBW") == 0) {
    expectedType = 0xF8;
  } else if (strcmp(mode, "IMG256") == 0) {
    expectedType = 0x55;
  } else {
    Serial.println("ERR UNKNOWN_TYPE");
    return false;
  }

  uint8_t marker;
  uint8_t type;
  if (!lireOctetUSB(marker) || !lireOctetUSB(type)) {
    Serial.println("ERR USB_TIMEOUT");
    return false;
  }
  if (marker != SOT) {
    Serial.println("ERR INVALID_SOT");
    return false;
  }
  if (type != expectedType) {
    Serial.println("ERR TYPE_MISMATCH");
    return false;
  }

  transmissionTerminee = false;
  bitIndex = -1;
  envoyerOctet(marker);
  envoyerOctet(type);

  uint32_t payloadSize;
  uint16_t rows = 0;
  uint16_t columns = 0;
  if (strcmp(mode, "TXT") == 0) {
    uint16_t textSize;
    if (!lireUInt16LEUSB(textSize)) {
      Serial.println("ERR USB_TIMEOUT");
      return false;
    }
    envoyerOctet(static_cast<uint8_t>(textSize & 0xFF));
    envoyerOctet(static_cast<uint8_t>(textSize >> 8));
    payloadSize = textSize;
  } else {
    if (!lireUInt16LEUSB(rows) || !lireUInt16LEUSB(columns)) {
      Serial.println("ERR USB_TIMEOUT");
      return false;
    }
    if (rows == 0 || columns == 0 ||
        rows > MAX_IMAGE_DIMENSION || columns > MAX_IMAGE_DIMENSION) {
      Serial.println("ERR IMAGE_DIMENSIONS");
      return false;
    }
    envoyerOctet(static_cast<uint8_t>(rows & 0xFF));
    envoyerOctet(static_cast<uint8_t>(rows >> 8));
    envoyerOctet(static_cast<uint8_t>(columns & 0xFF));
    envoyerOctet(static_cast<uint8_t>(columns >> 8));

    const uint32_t pixels = static_cast<uint32_t>(rows) * columns;
    payloadSize = strcmp(mode, "IMGBW") == 0
                      ? (pixels + 7) / 8
                      : 768 + pixels;
  }

  uint8_t finalPayloadByte = 0;
  for (uint32_t index = 0; index < payloadSize; ++index) {
    uint8_t value;
    if (!lireOctetUSB(value)) {
      Serial.println("ERR USB_TIMEOUT");
      return false;
    }
    if (strcmp(mode, "IMGBW") == 0 && index + 1 == payloadSize) {
      finalPayloadByte = value;
    }
    envoyerOctet(value);
  }

  if (strcmp(mode, "IMGBW") == 0) {
    const uint32_t pixelCount = static_cast<uint32_t>(rows) * columns;
    const uint8_t remainder = pixelCount % 8;
    if (remainder != 0) {
      const uint8_t unusedMask = (1U << (8 - remainder)) - 1;
      if ((finalPayloadByte & unusedMask) != 0) {
        Serial.println("ERR BW_PADDING");
        return false;
      }
    }
  }

  uint8_t endMarker;
  if (!lireOctetUSB(endMarker)) {
    Serial.println("ERR USB_TIMEOUT");
    return false;
  }
  if (endMarker != EOT) {
    Serial.println("ERR INVALID_EOT");
    return false;
  }
  envoyerOctet(endMarker);
  mettreEnFile(END_OF_TRANSFER, true);

  while (!transmissionTerminee) {
    yield();
  }
  Serial.println("DONE");
  return true;
}

void reglerDebit(const char *argument) {
  char *fin = nullptr;
  const unsigned long debit = strtoul(argument, &fin, 10);
  if (argument[0] == '\0' || fin == argument || *fin != '\0' ||
      debit < MIN_BIT_RATE || debit > MAX_BIT_RATE) {
    Serial.println("ERR INVALID_BAUD");
    return;
  }

  Timer3.stop();
  Timer3.setFrequency(static_cast<double>(debit));
  Timer3.start();
  Serial.print("OK BAUD ");
  Serial.println(static_cast<uint32_t>(debit));
}

void traiterCommande(char *ligne) {
  if (strncmp(ligne, "SET_BAUD:", 9) == 0) {
    reglerDebit(ligne + 9);
  } else if (strcmp(ligne, "SEND_TXT") == 0) {
    Serial.println("READY");
    transmettreTrame("TXT");
  } else if (strcmp(ligne, "SEND_IMGBW") == 0) {
    Serial.println("READY");
    transmettreTrame("IMGBW");
  } else if (strcmp(ligne, "SEND_IMG256") == 0) {
    Serial.println("READY");
    transmettreTrame("IMG256");
  } else if (ligne[0] != '\0') {
    Serial.println("ERR UNKNOWN_COMMAND");
  }
}

void traiterOctetCommande(uint8_t valeur) {
  if (valeur == '\r') {
    return;
  }

  if (valeur == '\n') {
    if (commandeTropLongue) {
      Serial.println("ERR COMMAND_TOO_LONG");
    } else {
      commande[longueurCommande] = '\0';
      traiterCommande(commande);
    }
    longueurCommande = 0;
    commandeTropLongue = false;
    return;
  }

  if (static_cast<size_t>(longueurCommande) + 1 >= sizeof(commande)) {
    commandeTropLongue = true;
    return;
  }
  commande[longueurCommande++] = static_cast<char>(valeur);
}

void configurerPorteuse() {
  // D6 / PC24 is the Due PWM output for channel 7; 84 MHz / 840 = 100 kHz.
  pmc_enable_periph_clk(ID_PWM);
  PWM->PWM_CLK = PWM_CLK_DIVA(1) | PWM_CLK_PREA(0);
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC24, PIO_DEFAULT);

  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CMR = PWM_CMR_CPRE_CLKA;
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CPRD = PWM_PERIOD;
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CDTY = 0;
  PWM->PWM_ENA = 1U << PWM_CHANNEL;
}

}  // namespace

void envoyerOctet(byte donnee) {
  mettreEnFile(donnee);
}

void txBegin() {
  configurerPorteuse();

  Timer3.attachInterrupt(transmettreBit);
  Timer3.setFrequency(DEFAULT_BIT_RATE);
  Timer3.start();

  Serial.println("READY CPL");
}

void txProcess() {
  while (Serial.available() > 0) {
    traiterOctetCommande(static_cast<uint8_t>(Serial.read()));
  }
}

void setup() {
  Serial.begin(115200);
  txBegin();
}

void loop() {
  txProcess();
}
