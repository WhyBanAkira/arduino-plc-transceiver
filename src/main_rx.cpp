#include <Arduino.h>
#include <DueTimer.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint32_t DEFAULT_BIT_RATE = 1000;
constexpr uint32_t MIN_BIT_RATE = 1000;
constexpr uint32_t MAX_BIT_RATE = 50000;
constexpr uint8_t RECEIVER_PIN = 2;
constexpr uint32_t RECEIVER_MASK = 1UL << 25;
constexpr uint16_t RX_BUFFER_SIZE = 512;
constexpr uint16_t RX_BUFFER_MASK = RX_BUFFER_SIZE - 1;
constexpr uint16_t MAX_IMAGE_DIMENSION = 320;
constexpr uint8_t SOT = 0xF0;
constexpr uint8_t EOT = 0x0F;
constexpr uint8_t TYPE_TEXT = 0x8F;
constexpr uint8_t TYPE_BW = 0xF8;
constexpr uint8_t TYPE_256 = 0x55;

enum class SampleStage : uint8_t {
  Idle,
  Start,
  Data,
};

// La trame CPL est décodée octet par octet dans loop(), hors des interruptions.
enum class ProtocolStage : uint8_t {
  WaitSot,
  WaitType,
  ReadMetadata,
  ReadPayload,
  ReadEot,
};

volatile SampleStage sampleStage = SampleStage::Idle;
volatile bool receivingByte = false;
volatile uint8_t halfTicks = 0;
volatile uint8_t dataBitIndex = 0;
volatile uint8_t currentByte = 0;

volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxHead = 0;
volatile uint16_t rxTail = 0;

ProtocolStage protocolStage = ProtocolStage::WaitSot;
uint8_t frameType = 0;
uint8_t metadata[4] = {};
uint8_t metadataSize = 0;
uint8_t metadataRead = 0;
uint16_t rows = 0;
uint16_t columns = 0;
uint32_t payloadRemaining = 0;

char command[32];
uint8_t commandLength = 0;
bool commandTooLong = false;
uint32_t bitRate = DEFAULT_BIT_RATE;

inline bool inputIsHigh() {
  return (PIOB->PIO_PDSR & RECEIVER_MASK) != 0;
}

void resetByteSampler() {
  receivingByte = false;
  sampleStage = SampleStage::Idle;
  Timer4.stop();
}

// L’ISR ne fait que placer un octet complet dans un tampon circulaire;
// l’accès série USB et le traitement du protocole restent dans loop().
void enqueueReceivedByte(uint8_t value) {
  const uint16_t next = (rxHead + 1) & RX_BUFFER_MASK;
  if (next != rxTail) {
    rxBuffer[rxHead] = value;
    rxHead = next;
  }
}

void onStartBit() {
  if (receivingByte || !inputIsHigh()) {
    return;
  }

  // Le flanc montant marque le début; Timer4 est réglé à deux ticks par bit.
  receivingByte = true;
  sampleStage = SampleStage::Start;
  halfTicks = 0;
  dataBitIndex = 0;
  currentByte = 0;
  Timer4.start();
}

// Timer4 interrompt à 2x le débit : le premier tick vérifie le start au milieu
// du bit, puis un tick sur deux lit chaque bit au centre de sa période.
void sampleInput() {
  if (!receivingByte) {
    return;
  }

  if (sampleStage == SampleStage::Start) {
    if (!inputIsHigh()) {
      resetByteSampler();
      return;
    }
    sampleStage = SampleStage::Data;
    halfTicks = 0;
    return;
  }

  if (++halfTicks < 2) {
    return;
  }
  halfTicks = 0;

  if (dataBitIndex < 8) {
    // Les octets CPL sont transmis LSB en premier.
    if (inputIsHigh()) {
      currentByte |= static_cast<uint8_t>(1U << dataBitIndex);
    }
    ++dataBitIndex;
    return;
  }

  // Le stop bit doit être bas; tout autre niveau invalide l’octet.
  if (!inputIsHigh()) {
    enqueueReceivedByte(currentByte);
  }
  resetByteSampler();
}

void resetProtocol(uint8_t possibleSot = 0) {
  protocolStage = possibleSot == SOT
                      ? ProtocolStage::WaitType
                      : ProtocolStage::WaitSot;
  frameType = 0;
  metadataSize = 0;
  metadataRead = 0;
  rows = 0;
  columns = 0;
  payloadRemaining = 0;
}

void sendReceiveHeader(uint32_t payloadSize) {
  if (frameType == TYPE_TEXT) {
    Serial.print("RCV_TXT ");
    Serial.println(payloadSize);
  } else if (frameType == TYPE_BW) {
    Serial.print("RCV_IMGBW ");
    Serial.print(rows);
    Serial.print(' ');
    Serial.print(columns);
    Serial.print(' ');
    Serial.println(payloadSize);
  } else {
    Serial.print("RCV_IMG256 ");
    Serial.print(rows);
    Serial.print(' ');
    Serial.print(columns);
    Serial.print(' ');
    Serial.println(payloadSize);
  }
}

void finishMetadata() {
  if (frameType == TYPE_TEXT) {
    // TXT: taille non signée sur deux octets, poids faible d’abord.
    payloadRemaining = static_cast<uint16_t>(metadata[0]) |
                       (static_cast<uint16_t>(metadata[1]) << 8);
  } else {
    // Images: lignes et colonnes sont deux entiers little-endian de 16 bits.
    rows = static_cast<uint16_t>(metadata[0]) |
           (static_cast<uint16_t>(metadata[1]) << 8);
    columns = static_cast<uint16_t>(metadata[2]) |
              (static_cast<uint16_t>(metadata[3]) << 8);
    if (rows == 0 || columns == 0 ||
        rows > MAX_IMAGE_DIMENSION || columns > MAX_IMAGE_DIMENSION) {
      resetProtocol();
      return;
    }

    const uint32_t pixels = static_cast<uint32_t>(rows) * columns;
    payloadRemaining = frameType == TYPE_BW ? (pixels + 7) / 8
                                             : 768 + pixels;
  }

  // L’entête USB annonce la taille exacte avant le flux binaire brut.
  sendReceiveHeader(payloadRemaining);
  protocolStage = payloadRemaining == 0 ? ProtocolStage::ReadEot
                                         : ProtocolStage::ReadPayload;
}

// La machine de protocole recherche SOT, lit le type et les métadonnées,
// relaie exactement le payload binaire au PC, puis vérifie l’octet EOT.
void consumeReceivedByte(uint8_t value) {
  switch (protocolStage) {
    case ProtocolStage::WaitSot:
      if (value == SOT) {
        protocolStage = ProtocolStage::WaitType;
      }
      break;

    case ProtocolStage::WaitType:
      if (value == TYPE_TEXT || value == TYPE_BW || value == TYPE_256) {
        frameType = value;
        metadataSize = frameType == TYPE_TEXT ? 2 : 4;
        metadataRead = 0;
        protocolStage = ProtocolStage::ReadMetadata;
      } else {
        resetProtocol(value);
      }
      break;

    case ProtocolStage::ReadMetadata:
      metadata[metadataRead++] = value;
      if (metadataRead == metadataSize) {
        finishMetadata();
      }
      break;

    case ProtocolStage::ReadPayload:
      Serial.write(value);
      if (--payloadRemaining == 0) {
        protocolStage = ProtocolStage::ReadEot;
      }
      break;

    case ProtocolStage::ReadEot:
      if (value == EOT) {
        Serial.println("RCV_OK");
        resetProtocol();
      } else {
        Serial.println("RCV_ERR_EOT");
        resetProtocol(value);
      }
      break;
  }
}

bool readReceivedByte(uint8_t &value) {
  // Protéger brièvement les index partagés entre loop() et l’ISR.
  noInterrupts();
  if (rxTail == rxHead) {
    interrupts();
    return false;
  }
  value = rxBuffer[rxTail];
  rxTail = (rxTail + 1) & RX_BUFFER_MASK;
  interrupts();
  return true;
}

void configureBitRate(uint32_t requestedRate) {
  if (requestedRate < MIN_BIT_RATE || requestedRate > MAX_BIT_RATE) {
    Serial.println("ERR INVALID_BAUD");
    return;
  }
  if (receivingByte) {
    Serial.println("ERR BUSY");
    return;
  }

  Timer4.stop();
  bitRate = requestedRate;
  Timer4.setFrequency(static_cast<double>(bitRate) * 2.0);
  Serial.print("OK BAUD ");
  Serial.println(bitRate);
}

void processCommand(const char *line) {
  if (strncmp(line, "SET_BAUD:", 9) != 0) {
    if (line[0] != '\0') {
      Serial.println("ERR UNKNOWN_COMMAND");
    }
    return;
  }

  char *end = nullptr;
  const unsigned long requestedRate = strtoul(line + 9, &end, 10);
  if (line[9] == '\0' || end == line + 9 || *end != '\0' ||
      requestedRate > UINT32_MAX) {
    Serial.println("ERR INVALID_BAUD");
    return;
  }
  configureBitRate(static_cast<uint32_t>(requestedRate));
}

void consumeCommandByte(uint8_t value) {
  if (value == '\r') {
    return;
  }
  if (value == '\n') {
    if (commandTooLong) {
      Serial.println("ERR COMMAND_TOO_LONG");
    } else {
      command[commandLength] = '\0';
      processCommand(command);
    }
    commandLength = 0;
    commandTooLong = false;
    return;
  }
  if (static_cast<size_t>(commandLength) + 1 >= sizeof(command)) {
    commandTooLong = true;
    return;
  }
  command[commandLength++] = static_cast<char>(value);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(RECEIVER_PIN, INPUT);
  // L’ISR externe ne fait que démarrer l’échantillonnage temporisé.
  Timer4.attachInterrupt(sampleInput);
  Timer4.setFrequency(static_cast<double>(DEFAULT_BIT_RATE) * 2.0);
  Timer4.stop();
  attachInterrupt(digitalPinToInterrupt(RECEIVER_PIN), onStartBit, RISING);
  Serial.println("READY RCV");
}

void loop() {
  // Traiter d’abord les octets CPL mis en tampon par l’ISR du timer.
  uint8_t value;
  while (readReceivedByte(value)) {
    consumeReceivedByte(value);
  }

  // SET_BAUD est une commande ASCII USB; elle ne passe pas dans le décodeur CPL.
  while (Serial.available() > 0) {
    const int received = Serial.read();
    if (received >= 0) {
      consumeCommandByte(static_cast<uint8_t>(received));
    }
  }
}
