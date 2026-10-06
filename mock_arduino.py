import argparse
import sys
import time
from pathlib import Path

import serial
from PIL import Image

BAUD_RATE = 115200
MAX_IMAGE_DIMENSION = 320
SOT = 0xF0
EOT = 0x0F
MESSAGE_TYPES = {"TXT": 0x8F, "IMGBW": 0xF8, "IMG256": 0x55}
DEFAULT_FILE = Path(__file__).resolve().with_name("simu_cpl_tx.bin")
OUTPUT_IMAGE = Path(__file__).resolve().with_name("output_test.png")


class ProtocolError(ValueError):
    pass


class ByteReader:
    def __init__(self, data):
        self.data = data
        self.offset = 0

    def read_line(self, field):
        end = self.data.find(b"\n", self.offset)
        if end < 0:
            raise ProtocolError(f"Commande {field} sans saut de ligne.")
        result = self.data[self.offset:end]
        self.offset = end + 1
        try:
            return result.decode("ascii")
        except UnicodeDecodeError as error:
            raise ProtocolError(f"Commande {field} non ASCII.") from error

    def remaining(self):
        return self.data[self.offset :]


def expected_payload_size(mode, rows, columns):
    if rows < 1 or columns < 1:
        raise ProtocolError("Les dimensions de l’image doivent être positives.")
    if rows > MAX_IMAGE_DIMENSION or columns > MAX_IMAGE_DIMENSION:
        raise ProtocolError(
            f"Image trop grande : {rows} × {columns}; "
            f"maximum {MAX_IMAGE_DIMENSION} × {MAX_IMAGE_DIMENSION}."
        )
    pixels = rows * columns
    return (pixels + 7) // 8 if mode == "IMGBW" else 768 + pixels


def frame_size_from_header(mode, frame_prefix):
    if mode not in MESSAGE_TYPES:
        raise ProtocolError(f"Mode inconnu : {mode}.")
    if len(frame_prefix) < 2:
        raise ProtocolError("Trame CPL trop courte pour son en-tête.")
    if frame_prefix[0] != SOT:
        raise ProtocolError("Marqueur SOT (0xF0) absent ou incorrect.")
    if frame_prefix[1] != MESSAGE_TYPES[mode]:
        raise ProtocolError(
            f"Type CPL incorrect : 0x{frame_prefix[1]:02X}, "
            f"attendu 0x{MESSAGE_TYPES[mode]:02X}."
        )

    metadata_size = 2 if mode == "TXT" else 4
    if len(frame_prefix) < 2 + metadata_size:
        raise ProtocolError("Trame CPL tronquée avant la taille ou les dimensions.")
    metadata = frame_prefix[2 : 2 + metadata_size]
    if mode == "TXT":
        payload_size = int.from_bytes(metadata, "little")
    else:
        rows = int.from_bytes(metadata[:2], "little")
        columns = int.from_bytes(metadata[2:], "little")
        payload_size = expected_payload_size(mode, rows, columns)
    return 2 + metadata_size + payload_size + 1


def validate_frame(frame, mode):
    expected_size = frame_size_from_header(mode, frame)
    if len(frame) != expected_size:
        raise ProtocolError(
            f"Taille de trame incorrecte : {len(frame)} octets, "
            f"{expected_size} attendus."
        )
    if frame[-1] != EOT:
        raise ProtocolError("Marqueur EOT (0x0F) absent ou incorrect.")

    if mode == "TXT":
        data_size = int.from_bytes(frame[2:4], "little")
        try:
            text = frame[4:-1].decode("utf-8")
        except UnicodeDecodeError as error:
            raise ProtocolError("Le texte reçu n’est pas un UTF-8 valide.") from error
        print(f"Texte reçu ({data_size} octets UTF-8) : {text}")
        return

    rows = int.from_bytes(frame[2:4], "little")
    columns = int.from_bytes(frame[4:6], "little")
    payload = frame[6:-1]

    if mode == "IMGBW":
        pixel_count = rows * columns
        remainder = pixel_count % 8
        if remainder:
            unused_mask = (1 << (8 - remainder)) - 1
            if payload[-1] & unused_mask:
                raise ProtocolError(
                    "Les bits inutilisés du dernier octet BW doivent être à zéro."
                )

        pixels = [
            255 if (payload[index // 8] >> (7 - index % 8)) & 1 else 0
            for index in range(pixel_count)
        ]
        image = Image.new("L", (columns, rows))
        image.putdata(pixels)
    else:
        palette = payload[:768]
        indices = payload[768:]
        image = Image.frombytes("P", (columns, rows), indices)
        image.putpalette(list(palette))
        image = image.convert("RGB")

    image.save(OUTPUT_IMAGE, format="PNG")
    print(
        f"Image {mode} reçue : {rows} lignes × {columns} colonnes, "
        f"{len(payload)} octets de payload."
    )
    print(f"Image reconstruite : {OUTPUT_IMAGE}")


def read_file_transfer(path):
    data = path.read_bytes()
    reader = ByteReader(data)
    baud_command = reader.read_line("SET_BAUD")
    print(f"Commande reçue : {baud_command}")
    if not baud_command.startswith("SET_BAUD:"):
        raise ProtocolError("La première commande doit être SET_BAUD:<débit>.")
    try:
        bit_rate = int(baud_command.partition(":")[2])
    except ValueError as error:
        raise ProtocolError("Débit série non numérique.") from error
    if not 1000 <= bit_rate <= 50000:
        raise ProtocolError("Le débit doit être compris entre 1000 et 50000 bit/s.")

    send_command = reader.read_line("SEND")
    print(f"Commande reçue : {send_command}")
    if not send_command.startswith("SEND_"):
        raise ProtocolError("La deuxième commande doit être SEND_<type>.")
    mode = send_command[5:]
    if mode not in MESSAGE_TYPES:
        raise ProtocolError(f"Commande de transfert inconnue : {send_command}.")

    frame = reader.remaining()
    validate_frame(frame, mode)
    print(f"Configuration du firmware simulée : {bit_rate} bit/s.")
    print(f"Trame CPL exacte présente dans le fichier : {len(frame)} octets.")


def read_exact(port, size, deadline, field):
    chunks = bytearray()
    while len(chunks) < size:
        if time.monotonic() >= deadline:
            raise ProtocolError(f"Délai dépassé pendant la lecture de {field}.")
        chunk = port.read(size - len(chunks))
        if chunk:
            chunks.extend(chunk)
    return bytes(chunks)


def read_serial_transfer(port, mode, bit_rate):
    estimated_timeout = 60 + (320 * 320 + 768 + 16) * 10 / bit_rate
    deadline = time.monotonic() + estimated_timeout
    header = read_exact(port, 2, deadline, "l’en-tête CPL")
    if header[0] != SOT:
        raise ProtocolError("Marqueur SOT (0xF0) absent ou incorrect.")
    if header[1] != MESSAGE_TYPES[mode]:
        raise ProtocolError("Le type de la trame ne correspond pas à la commande SEND.")

    metadata_size = 2 if mode == "TXT" else 4
    metadata = read_exact(port, metadata_size, deadline, "la taille ou les dimensions")
    prefix = header + metadata
    total_size = frame_size_from_header(mode, prefix)
    remainder = read_exact(
        port, total_size - len(prefix), deadline, "le payload et le marqueur EOT"
    )
    validate_frame(prefix + remainder, mode)


def run_serial(port_name):
    print(f"Mock Arduino à l’écoute sur {port_name} ({BAUD_RATE} bauds).")
    print("Connecte l’autre extrémité du port série virtuel depuis l’interface.")
    with serial.Serial(port_name, BAUD_RATE, timeout=0.5) as port:
        bit_rate = 1000
        while True:
            raw_command = port.readline()
            if not raw_command:
                continue
            try:
                command = raw_command.decode("ascii").strip()
                print(f"Commande reçue : {command}")
                if command.startswith("SET_BAUD:"):
                    bit_rate = int(command.partition(":")[2])
                    if not 1000 <= bit_rate <= 50000:
                        raise ProtocolError("Débit hors limites.")
                    port.write(f"OK BAUD {bit_rate}\n".encode("ascii"))
                elif command.startswith("SEND_"):
                    mode = command[5:]
                    if mode not in MESSAGE_TYPES:
                        raise ProtocolError(f"Commande inconnue : {command}.")
                    port.write(b"READY\n")
                    read_serial_transfer(port, mode, bit_rate)
                    port.write(b"DONE\n")
                else:
                    raise ProtocolError(f"Commande inconnue : {command}.")
            except (UnicodeDecodeError, ValueError, OSError, ProtocolError) as error:
                print(f"ERREUR : {error}", file=sys.stderr)
                try:
                    port.write(f"ERR {error}\n".encode("ascii", errors="replace"))
                except serial.SerialException:
                    return 1
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Simule le récepteur Arduino CPL pour tester l’émetteur."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--file",
        nargs="?",
        const=str(DEFAULT_FILE),
        help=f"lit un fichier USB simulé (par défaut : {DEFAULT_FILE.name})",
    )
    group.add_argument(
        "--port",
        help="écoute un port série virtuel connecté à l’interface GUI",
    )
    args = parser.parse_args()

    try:
        if args.port:
            return run_serial(args.port)
        file_path = Path(args.file) if args.file else DEFAULT_FILE
        read_file_transfer(file_path)
        return 0
    except (OSError, ProtocolError) as error:
        print(f"Échec du test : {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nArrêt du simulateur.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
