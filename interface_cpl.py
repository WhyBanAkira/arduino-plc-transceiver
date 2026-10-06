import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import serial
from PIL import Image, ImageOps, ImageTk
from serial.tools import list_ports
from tkinterdnd2 import DND_FILES, TkinterDnD

BAUD_RATE = 115200
ARDUINO_VENDOR_IDS = {0x2341, 0x2A03}
MAX_IMAGE_DIMENSION = 320
MAX_TEXT_BYTES = 0xFFFF
USB_CHUNK_SIZE = 32
TYPE_TEXT = "TXT"
TYPE_BW = "IMGBW"
TYPE_256 = "IMG256"
ROLE_TRANSMITTER = "Mode Émetteur"
ROLE_RECEIVER = "Mode Récepteur"
SOT = 0xF0
EOT = 0x0F
MESSAGE_TYPES = {TYPE_TEXT: 0x8F, TYPE_BW: 0xF8, TYPE_256: 0x55}


def detect_arduino_port():
    ports = list(list_ports.comports())
    arduino_ports = [
        port for port in ports if port.vid in ARDUINO_VENDOR_IDS
    ]
    if len(arduino_ports) == 1:
        return arduino_ports[0].device
    if len(arduino_ports) > 1:
        raise RuntimeError(
            "Plusieurs cartes Arduino ont été détectées. "
            "Sélectionne le port série voulu."
        )
    raise RuntimeError(
        "Aucune carte Arduino n'a été détectée. "
        "Branche la carte ou sélectionne son port manuellement."
    )


def build_cpl_frame(mode, body):
    if mode not in MESSAGE_TYPES:
        raise ValueError(f"Mode de message inconnu : {mode}.")
    return bytes((SOT, MESSAGE_TYPES[mode])) + body + bytes((EOT,))


def encode_image(path, mode, resize_image):
    with Image.open(path) as source:
        image = ImageOps.exif_transpose(source)
        image.load()

    if image.width < 1 or image.height < 1:
        raise ValueError("L'image ne contient aucun pixel.")

    if image.width > MAX_IMAGE_DIMENSION or image.height > MAX_IMAGE_DIMENSION:
        if not resize_image:
            raise ValueError("Le redimensionnement a été annulé.")
        image.thumbnail((MAX_IMAGE_DIMENSION, MAX_IMAGE_DIMENSION), Image.Resampling.LANCZOS)

    if "A" in image.getbands() or "transparency" in image.info:
        rgba = image.convert("RGBA")
        background = Image.new("RGBA", rgba.size, (255, 255, 255, 255))
        image = Image.alpha_composite(background, rgba).convert("RGB")
    else:
        image = image.convert("RGB")

    rows, columns = image.height, image.width
    dimensions = rows.to_bytes(2, "little") + columns.to_bytes(2, "little")

    if mode == TYPE_BW:
        monochrome = image.convert("L").point(
            lambda value: 255 if value >= 128 else 0
        )
        pixels = monochrome.tobytes()
        packed = bytearray((len(pixels) + 7) // 8)
        for index, pixel in enumerate(pixels):
            if pixel:
                packed[index // 8] |= 1 << (7 - index % 8)
        return dimensions + packed

    quantized = image.quantize(colors=256, method=Image.Quantize.FASTOCTREE)
    palette = (quantized.getpalette() or [])[:768]
    palette.extend([0] * (768 - len(palette)))
    return dimensions + bytes(palette) + quantized.tobytes()


class CplTransmitterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Interface CPL — Arduino Due")
        self.root.minsize(900, 550)
        self.root.geometry("1100x680")

        self.events = queue.Queue()
        self.sending = False
        self.listening = False
        self.receiver_failed = False
        self.receiver_stop = threading.Event()
        self.receiver_thread = None
        self.image_path = None
        self.received_photo = None
        self.role_var = tk.StringVar(value=ROLE_TRANSMITTER)
        self.mode_var = tk.StringVar(value=TYPE_TEXT)
        self.port_var = tk.StringVar()
        self.rate_var = tk.StringVar(value="1000")
        self.status_var = tk.StringVar(value="Prêt.")
        self.image_info_var = tk.StringVar(
            value="Glisse une image ici ou sélectionne un fichier."
        )

        self._build_ui()
        self._refresh_ports()
        self._layout_role_views()
        self._set_mode()
        self.root.after(100, self._process_events)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def _build_ui(self):
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Listening.TButton", foreground="#c62828")

        container = ttk.Frame(self.root, padding=20)
        container.pack(fill="both", expand=True)
        container.columnconfigure(1, weight=1)
        container.rowconfigure(5, weight=1)

        ttk.Label(
            container,
            text="Interface CPL",
            font=("Segoe UI", 20, "bold"),
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 4))
        ttk.Label(
            container,
            text="Émission et réception CPL via une Arduino Due",
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(0, 18))

        ttk.Label(container, text="Rôle").grid(row=2, column=0, sticky="w")
        self.role_box = ttk.Combobox(
            container,
            textvariable=self.role_var,
            values=(ROLE_TRANSMITTER, ROLE_RECEIVER),
            state="readonly",
            width=24,
        )
        self.role_box.grid(row=2, column=1, sticky="ew", padx=10, pady=5)
        self.role_box.bind("<<ComboboxSelected>>", self._set_role)

        ttk.Label(container, text="Port série").grid(row=3, column=0, sticky="w")
        self.port_box = ttk.Combobox(
            container, textvariable=self.port_var, state="normal", width=24
        )
        self.port_box.grid(row=3, column=1, sticky="ew", padx=10, pady=5)
        self.refresh_button = ttk.Button(
            container, text="Actualiser", command=self._refresh_ports
        )
        self.refresh_button.grid(row=3, column=2, sticky="ew", pady=5)

        ttk.Label(container, text="Débit (bit/s)").grid(
            row=4, column=0, sticky="w"
        )
        self.rate_box = ttk.Combobox(
            container,
            textvariable=self.rate_var,
            values=("1000", "5000", "10000", "25000", "50000"),
            state="normal",
            width=24,
        )
        self.rate_box.grid(row=4, column=1, sticky="ew", padx=10, pady=5)

        content = ttk.LabelFrame(container, text="Données à transmettre", padding=12)
        content.grid(row=5, column=0, columnspan=3, sticky="nsew", pady=(15, 10))
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(1, weight=1)
        self.content = content

        mode_row = ttk.Frame(content)
        self.mode_row = mode_row
        mode_row.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        ttk.Label(mode_row, text="Mode").pack(side="left")
        self.mode_box = ttk.Combobox(
            mode_row,
            textvariable=self.mode_var,
            values=(TYPE_TEXT, TYPE_BW, TYPE_256),
            state="readonly",
            width=18,
        )
        self.mode_box.pack(side="left", padx=10)
        self.mode_box.bind("<<ComboboxSelected>>", lambda _event: self._set_mode())

        self.send_frame = ttk.LabelFrame(
            content, text="À transmettre", padding=8
        )
        self.send_frame.rowconfigure(0, weight=1)
        self.send_frame.columnconfigure(0, weight=1)

        self.text_frame = ttk.Frame(self.send_frame)
        self.text_frame.grid(row=0, column=0, sticky="nsew")
        self.text_frame.rowconfigure(0, weight=1)
        self.text_frame.columnconfigure(0, weight=1)
        self.text_input = tk.Text(
            self.text_frame, height=10, wrap="word", undo=True, font=("Segoe UI", 11)
        )
        self.text_input.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(
            self.text_frame, orient="vertical", command=self.text_input.yview
        )
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.text_input.configure(yscrollcommand=scrollbar.set)

        self.image_frame = ttk.Frame(self.send_frame)
        self.image_frame.grid(row=0, column=0, sticky="nsew")
        self.image_frame.columnconfigure(0, weight=1)
        self.drop_target = ttk.Label(
            self.image_frame,
            textvariable=self.image_info_var,
            anchor="center",
            justify="center",
            relief="groove",
            padding=30,
        )
        self.drop_target.grid(row=0, column=0, sticky="nsew", pady=(0, 10))
        self.drop_target.drop_target_register(DND_FILES)
        self.drop_target.dnd_bind("<<Drop>>", self._on_drop)
        ttk.Button(
            self.image_frame, text="Choisir une image…", command=self._choose_image
        ).grid(row=1, column=0, sticky="e")

        self.receive_frame = ttk.LabelFrame(
            content, text="Données reçues", padding=8
        )
        self.receive_frame.rowconfigure(0, weight=1)
        self.receive_frame.columnconfigure(0, weight=1)
        self.receive_frame.columnconfigure(1, weight=1)
        self.received_text_frame = ttk.Frame(self.receive_frame)
        self.received_text_frame.grid(
            row=0, column=0, columnspan=2, sticky="nsew"
        )
        self.received_text_frame.rowconfigure(0, weight=1)
        self.received_text_frame.columnconfigure(0, weight=1)
        self.received_text = tk.Text(
            self.received_text_frame,
            height=10,
            wrap="word",
            state="disabled",
            font=("Segoe UI", 10),
        )
        self.received_text.grid(
            row=0, column=0, sticky="nsew"
        )
        self.received_scrollbar = ttk.Scrollbar(
            self.received_text_frame,
            orient="vertical",
            command=self.received_text.yview,
        )
        self.received_scrollbar.grid(row=0, column=1, sticky="ns")
        self.received_text.configure(yscrollcommand=self.received_scrollbar.set)
        self.received_image = ttk.Label(
            self.receive_frame,
            text="Les images reçues apparaîtront ici.",
            anchor="center",
            relief="groove",
        )
        self.received_image.grid(
            row=0, column=0, columnspan=2, sticky="nsew"
        )
        self.received_image.grid_remove()

        footer = ttk.Frame(container)
        footer.grid(row=6, column=0, columnspan=3, sticky="ew")
        footer.columnconfigure(0, weight=1)
        self.status_label = ttk.Label(
            footer, textvariable=self.status_var, wraplength=520
        )
        self.status_label.grid(row=0, column=0, sticky="w")
        self.send_button = ttk.Button(
            footer, text="Envoyer", command=self._start_send
        )
        self.send_button.grid(row=0, column=1, sticky="e", padx=(12, 0))

    def _set_role(self, _event=None):
        self._layout_role_views()
        if self.role_var.get() == ROLE_RECEIVER:
            self.content.configure(text="Réception")
            self.send_button.configure(
                text="Démarrer l'écoute",
                command=self._toggle_receiver,
                style="TButton",
            )
        else:
            self._stop_receiver()
            self.content.configure(text="Données à transmettre")
            self.send_button.configure(
                text="Envoyer", command=self._start_send, style="TButton"
            )
            self._set_mode()

    def _layout_role_views(self):
        self.mode_row.grid_remove()
        self.send_frame.grid_remove()
        self.receive_frame.grid_remove()

        role = self.role_var.get()
        if role == ROLE_RECEIVER:
            self.receive_frame.grid(
                row=1, column=0, columnspan=2, sticky="nsew"
            )
        else:
            self.mode_row.grid(
                row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10)
            )
            self.send_frame.grid(
                row=1, column=0, columnspan=2, sticky="nsew"
            )

    def _start_receiver(self):
        if self.listening:
            return
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("Sélectionne le port série de la Due réceptrice.")
            return
        try:
            bit_rate = int(self.rate_var.get())
        except ValueError:
            self.status_var.set("Le débit doit être un nombre entier.")
            return
        if not 1000 <= bit_rate <= 50000:
            self.status_var.set("Le débit doit être compris entre 1000 et 50000 bit/s.")
            return

        self.receiver_stop.clear()
        self.receiver_failed = False
        self.listening = True
        self.send_button.configure(
            text="Arrêter l'écoute",
            command=self._toggle_receiver,
            style="Listening.TButton",
            state="normal",
        )
        self.refresh_button.configure(state="disabled")
        self.role_box.configure(state="disabled")
        self.port_box.configure(state="disabled")
        self.rate_box.configure(state="disabled")
        self.status_var.set(f"Statut : Connexion au récepteur sur {port}…")
        self.receiver_thread = threading.Thread(
            target=self._receive_worker,
            args=(port, bit_rate),
            daemon=True,
        )
        self.receiver_thread.start()

    def _toggle_receiver(self):
        if self.listening:
            self._stop_receiver()
        else:
            self._start_receiver()

    def _stop_receiver(self):
        if not self.listening:
            return
        self.receiver_stop.set()
        self.listening = False
        self.send_button.configure(state="disabled")
        self.status_var.set("Arrêt de l’écoute…")

    def _receive_worker(self, port, bit_rate):
        try:
            with serial.Serial(port, BAUD_RATE, timeout=0.2) as arduino:
                if self.receiver_stop.wait(2):
                    return
                arduino.reset_input_buffer()
                arduino.write(f"SET_BAUD:{bit_rate}\n".encode("ascii"))
                self._expect_reply(
                    arduino,
                    f"OK BAUD {bit_rate}",
                    stop_event=self.receiver_stop,
                )
                if self.receiver_stop.is_set():
                    return
                self.events.put(
                    (
                        "receive_status",
                        f"Statut : Écoute continue sur le port {port}.",
                    )
                )

                while not self.receiver_stop.is_set():
                    if not arduino.is_open:
                        raise RuntimeError("La connexion série a été fermée.")
                    raw_header = arduino.readline()
                    if not raw_header:
                        continue
                    try:
                        header = raw_header.decode("ascii").strip()
                    except UnicodeDecodeError as error:
                        raise RuntimeError("Entête USB de réception non ASCII.") from error
                    if header.startswith("ERR "):
                        self.events.put(("receive_error", f"Erreur récepteur : {header[4:]}"))
                        continue
                    if header.startswith("RCV_"):
                        self._read_received_frame(arduino, header, bit_rate)
        except (
            serial.SerialException,
            TypeError,
            OSError,
            RuntimeError,
            ValueError,
        ) as error:
            if not self.receiver_stop.is_set():
                self.events.put(("receive_error", str(error)))
        finally:
            self.events.put(("receive_stopped", None))

    def _read_received_frame(self, arduino, header, bit_rate):
        fields = header.split()
        mode_by_header = {
            "RCV_TXT": TYPE_TEXT,
            "RCV_IMGBW": TYPE_BW,
            "RCV_IMG256": TYPE_256,
        }
        mode = mode_by_header.get(fields[0])
        if mode is None:
            raise RuntimeError(f"Type de trame USB inconnu : {fields[0]}.")
        if mode == TYPE_TEXT:
            if len(fields) != 2:
                raise RuntimeError("Entête TXT invalide.")
            payload_size = int(fields[1])
            if not 0 <= payload_size <= MAX_TEXT_BYTES:
                raise RuntimeError("Longueur TXT reçue hors limites.")
            rows = columns = None
        else:
            if len(fields) != 4:
                raise RuntimeError("Entête d’image invalide.")
            rows, columns, payload_size = map(int, fields[1:])
            if not 1 <= rows <= MAX_IMAGE_DIMENSION or not 1 <= columns <= MAX_IMAGE_DIMENSION:
                raise RuntimeError("Dimensions d’image reçues hors limites.")
            pixels = rows * columns
            expected = (pixels + 7) // 8 if mode == TYPE_BW else 768 + pixels
            if payload_size != expected:
                raise RuntimeError("Longueur du payload image incompatible avec ses dimensions.")
        if not 0 <= payload_size <= 0xFFFF + 768 + MAX_IMAGE_DIMENSION**2:
            raise RuntimeError("Longueur de payload reçue hors limites.")

        payload = self._read_exact(
            arduino,
            payload_size,
            time.monotonic() + max(30, payload_size * 10 / bit_rate + 10),
        )
        status = arduino.readline().decode("ascii", errors="strict").strip()
        if status == "RCV_ERR_EOT":
            self.events.put(
                ("receive_rejected", "Trame rejetée : marqueur EOT incorrect.")
            )
            return False
        if status != "RCV_OK":
            raise RuntimeError(
                "La Due n’a pas confirmé la validation EOT de la trame."
                if not status
                else f"Trame reçue mais non validée : {status}."
            )
        result = self._decode_received(mode, payload, rows, columns)
        self.events.put(("received", result))
        return True

    def _decode_received(self, mode, payload, rows, columns):
        if mode == TYPE_TEXT:
            try:
                return mode, payload.decode("ascii")
            except UnicodeDecodeError as error:
                raise RuntimeError("Le texte reçu contient des octets non ASCII.") from error

        pixels_count = rows * columns
        if mode == TYPE_BW:
            remainder = pixels_count % 8
            if remainder and payload[-1] & ((1 << (8 - remainder)) - 1):
                raise RuntimeError("Bits de remplissage BW non nuls dans le dernier octet.")
            pixels = bytes(
                255 if (payload[index // 8] >> (7 - index % 8)) & 1 else 0
                for index in range(pixels_count)
            )
            image = Image.frombytes("L", (columns, rows), pixels)
        else:
            palette = payload[:768]
            indices = payload[768:]
            image = Image.frombytes("P", (columns, rows), indices)
            image.putpalette(list(palette))
            image = image.convert("RGB")
        return mode, image

    def _save_received(self, mode, data):
        try:
            if mode == TYPE_TEXT:
                path = filedialog.asksaveasfilename(
                    parent=self.root,
                    title="Enregistrer le texte reçu",
                    defaultextension=".txt",
                    initialfile="message_recu.txt",
                    filetypes=(("Fichiers texte", "*.txt"),),
                )
                if not path:
                    self.status_var.set("Texte reçu; enregistrement annulé.")
                    return
                Path(path).write_text(data, encoding="ascii")
            else:
                path = filedialog.asksaveasfilename(
                    parent=self.root,
                    title="Enregistrer l’image reçue",
                    defaultextension=".png",
                    initialfile="image_recue.png",
                    filetypes=(("Image PNG", "*.png"),),
                )
                if not path:
                    self.status_var.set("Image reçue; enregistrement annulé.")
                    return
                data.save(path, format="PNG")
        except (OSError, ValueError, tk.TclError) as error:
            messagebox.showerror(
                "Échec de l’enregistrement", str(error), parent=self.root
            )
            self.status_var.set("Impossible d’enregistrer le fichier reçu.")
            return
        self.status_var.set(f"Fichier reçu enregistré : {path}")

    def _read_exact(self, arduino, size, deadline):
        data = bytearray()
        while len(data) < size and not self.receiver_stop.is_set():
            if time.monotonic() >= deadline:
                raise RuntimeError("Délai dépassé pendant la réception du payload.")
            chunk = arduino.read(min(4096, size - len(data)))
            if chunk:
                data.extend(chunk)
        if self.receiver_stop.is_set():
            raise RuntimeError("Écoute arrêtée.")
        return bytes(data)

    def _refresh_ports(self):
        ports = list(list_ports.comports())
        values = [port.device for port in ports]
        self.port_box["values"] = values
        if self.port_var.get() in values:
            return
        try:
            self.port_var.set(detect_arduino_port())
        except RuntimeError:
            self.port_var.set(values[0] if len(values) == 1 else "")

    def _set_mode(self):
        if self.role_var.get() == ROLE_RECEIVER:
            return
        if self.mode_var.get() == TYPE_TEXT:
            self.image_frame.grid_remove()
            self.text_frame.grid()
        else:
            self.text_frame.grid_remove()
            self.image_frame.grid()

    def _choose_image(self):
        path = filedialog.askopenfilename(
            title="Sélectionner une image",
            filetypes=(
                ("Images", "*.png *.jpg *.jpeg *.bmp *.gif *.tif *.tiff *.webp"),
                ("Tous les fichiers", "*.*"),
            ),
        )
        if path:
            self._set_image(path)

    def _on_drop(self, event):
        paths = self.root.tk.splitlist(event.data)
        if paths:
            self._set_image(paths[0])
        return event.action

    def _set_image(self, path):
        try:
            with Image.open(path) as image:
                width, height = image.size
            self.image_path = str(Path(path))
            self.image_info_var.set(
                f"{Path(path).name}\n{width} × {height} pixels"
                + (
                    f" (réduction à {MAX_IMAGE_DIMENSION} × "
                    f"{MAX_IMAGE_DIMENSION} proposée à l’envoi)"
                    if width > MAX_IMAGE_DIMENSION or height > MAX_IMAGE_DIMENSION
                    else ""
                )
            )
        except (OSError, ValueError) as error:
            messagebox.showerror("Image invalide", str(error), parent=self.root)

    def _collect_transfer(self):
        mode = self.mode_var.get()
        if mode == TYPE_TEXT:
            text = self.text_input.get("1.0", "end-1c")
            try:
                payload = text.encode("ascii")
            except UnicodeEncodeError as error:
                raise ValueError(
                    "Le mode TXT ne prend en charge que les caractères ASCII."
                ) from error
            if len(payload) > MAX_TEXT_BYTES:
                raise ValueError("Le texte dépasse la limite de 65 535 octets ASCII.")
            body = len(payload).to_bytes(2, "little") + payload
        else:
            if not self.image_path:
                raise ValueError("Sélectionne d’abord une image.")
            payload = encode_image(
                self.image_path,
                mode,
                resize_image=lambda: messagebox.askyesno(
                    "Redimensionner l’image",
                    f"L’image dépasse {MAX_IMAGE_DIMENSION} × "
                    f"{MAX_IMAGE_DIMENSION} pixels. La réduire automatiquement ?",
                    parent=self.root,
                ),
            )
            body = payload

        try:
            bit_rate = int(self.rate_var.get())
        except ValueError as error:
            raise ValueError("Le débit doit être un nombre entier.") from error
        if not 1000 <= bit_rate <= 50000:
            raise ValueError("Le débit doit être compris entre 1000 et 50000 bit/s.")
        return mode, bit_rate, build_cpl_frame(mode, body)

    def _start_send(self):
        if self.sending:
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror(
                "Port série manquant",
                "Branche l’Arduino ou sélectionne son port série.",
                parent=self.root,
            )
            return
        try:
            mode, bit_rate, frame = self._collect_transfer()
        except (OSError, ValueError) as error:
            messagebox.showerror("Données invalides", str(error), parent=self.root)
            return

        estimated_bytes = len(frame)
        seconds = estimated_bytes * 10 / bit_rate
        if not messagebox.askyesno(
            "Confirmer l’envoi",
            f"Transmettre {len(frame):,} octets en mode {mode} à "
            f"{bit_rate:,} bit/s ?\nDurée minimale estimée : "
            f"{self._format_duration(seconds)}.",
            parent=self.root,
        ):
            return

        self.sending = True
        self.send_button.configure(state="disabled")
        self.refresh_button.configure(state="disabled")
        self.role_box.configure(state="disabled")
        self.status_var.set("Connexion à l’Arduino…")
        worker = threading.Thread(
            target=self._send_worker,
            args=(port, bit_rate, mode, frame),
            daemon=True,
        )
        worker.start()

    def _send_worker(self, port, bit_rate, mode, frame):
        try:
            with serial.Serial(
                port,
                BAUD_RATE,
                timeout=0.25,
                write_timeout=None,
            ) as arduino:
                time.sleep(2)
                arduino.reset_input_buffer()

                arduino.write(f"SET_BAUD:{bit_rate}\n".encode("ascii"))
                self._expect_reply(arduino, f"OK BAUD {bit_rate}")

                arduino.write(f"SEND_{mode}\n".encode("ascii"))
                self._expect_reply(arduino, "READY")

                self.events.put(("status", "Transmission CPL en cours…"))
                for start in range(0, len(frame), USB_CHUNK_SIZE):
                    chunk = frame[start : start + USB_CHUNK_SIZE]
                    arduino.write(chunk)
                    time.sleep(len(chunk) * 8 / bit_rate)
                    if (start + len(chunk)) % 4096 < USB_CHUNK_SIZE:
                        sent = start + len(chunk)
                        self.events.put(
                            (
                                "status",
                                f"Données envoyées à l’Arduino : "
                                f"{sent * 100 // len(frame)} %",
                            )
                        )
                arduino.flush()
                self.events.put(
                    ("status", "Flux USB envoyé, émission CPL en cours…")
                )

                transfer_timeout = max(30, len(frame) * 10 / bit_rate + 30)
                self._expect_reply(arduino, "DONE", timeout=transfer_timeout)
            self.events.put(("success", "Transmission terminée."))
        except (serial.SerialException, TypeError, OSError, RuntimeError) as error:
            self.events.put(("error", str(error)))

    @staticmethod
    def _expect_reply(arduino, expected, timeout=30, stop_event=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not (
            stop_event is not None and stop_event.is_set()
        ):
            line = arduino.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("ERR "):
                raise RuntimeError(f"Erreur Arduino : {line[4:]}")
            if line == expected:
                return
        if stop_event is not None and stop_event.is_set():
            return
        raise RuntimeError(
            f"L’Arduino n’a pas répondu « {expected} ». "
            "Vérifie le firmware et la connexion série."
        )

    def _process_events(self):
        try:
            while True:
                event, message = self.events.get_nowait()
                if event == "status":
                    self.status_var.set(message)
                elif event == "receive_status":
                    self.status_var.set(message)
                elif event == "receive_error":
                    self.receiver_failed = True
                    self.status_var.set(message)
                    if not self.listening:
                        self._enable_receiver_controls()
                elif event == "receive_rejected":
                    self.status_var.set(message)
                elif event == "receive_stopped":
                    self.listening = False
                    self._enable_receiver_controls()
                    self.send_button.configure(state="normal")
                    if self.role_var.get() == ROLE_RECEIVER:
                        self.send_button.configure(
                            text="Démarrer l'écoute",
                            command=self._toggle_receiver,
                            style="TButton",
                        )
                        if not self.receiver_failed:
                            self.status_var.set("Statut : Écoute arrêtée.")
                elif event == "received":
                    mode, data = message
                    if mode == TYPE_TEXT:
                        self.received_image.grid_remove()
                        self.received_text_frame.grid()
                        self.received_text.configure(state="normal")
                        self.received_text.insert("end", data + "\n")
                        self.received_text.see("end")
                        self.received_text.configure(state="disabled")
                        self._save_received(mode, data)
                    else:
                        self.received_text_frame.grid_remove()
                        self.received_image.grid()
                        self.received_photo = ImageTk.PhotoImage(data)
                        self.received_image.configure(image=self.received_photo, text="")
                        self._save_received(mode, data)
                else:
                    self.sending = False
                    self.send_button.configure(state="normal")
                    self.refresh_button.configure(state="normal")
                    self.role_box.configure(state="readonly")
                    self.status_var.set(message)
                    if event == "success":
                        messagebox.showinfo("Transmission", message, parent=self.root)
                    else:
                        messagebox.showerror("Échec de transmission", message, parent=self.root)
        except queue.Empty:
            pass
        self.root.after(100, self._process_events)

    def _enable_receiver_controls(self):
        self.refresh_button.configure(state="normal")
        self.role_box.configure(state="readonly")
        self.port_box.configure(state="normal")
        self.rate_box.configure(state="normal")

    def _close(self):
        if self.sending:
            messagebox.showwarning(
                "Transmission en cours",
                "Attends la fin de la transmission avant de fermer l’application.",
                parent=self.root,
            )
            return
        if self.listening:
            self.receiver_stop.set()
        self.root.destroy()

    @staticmethod
    def _format_duration(seconds):
        total_seconds = int(seconds)
        minutes, seconds = divmod(total_seconds, 60)
        if minutes:
            return f"{minutes} min {seconds} s"
        return f"{seconds} s"


def main():
    root = TkinterDnD.Tk()
    CplTransmitterApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
