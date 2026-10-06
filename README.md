# Communication CPL — Arduino Due

Le projet utilise deux Arduino Due distincts : un émetteur et un récepteur,
reliés par les circuits de couplage CPL. L’application `interface_cpl.py`
propose uniquement les rôles Émetteur et Récepteur.

## Installation et lancement

```powershell
py -3 -m pip install -r requirements.txt
py -3 interface_cpl.py
```

Choisir le rôle, le port série et le débit CPL (1 000 à 50 000 bit/s). En mode
Émetteur, sélectionner Texte, IMGBW ou IMG256. En mode Récepteur, le programme
écoute en arrière-plan et propose d’enregistrer chaque message TXT validé comme
`.txt`, ou chaque image reconstruite comme `.png`.

Les images sont limitées à 320 × 320 pixels. Les images BW sont seuillées à 128,
parcourues ligne par ligne, avec le premier pixel dans le bit de poids fort et
un pixel blanc égal à 1. Les images IMG256 contiennent une palette RGB de 256
entrées (768 octets). Le texte est ASCII.

## Firmware

Les deux environnements PlatformIO compilent chacun un unique fichier source :

```powershell
pio run -e emitter
pio run -e receiver
pio run -e emitter -t upload
pio run -e receiver -t upload
```

Flasher `emitter` sur l’Arduino émettrice et `receiver` sur la réceptrice.
L’émetteur génère la porteuse PWM de 100 kHz sur la sortie D6 (PC24, canal PWM7)
et cadence les bits avec un timer matériel. Le récepteur utilise D2 comme entrée
du détecteur d’enveloppe, une interruption sur le start bit et un timer pour
échantillonner les bits. Maintenir D2 au niveau bas au repos avec une résistance
externe de rappel vers GND; le core Due ne fournit pas de pull-down interne.

La sortie Due est logique 3,3 V et ne doit jamais être reliée directement au
secteur ou à une prise électrique. Utiliser un circuit de couplage CPL isolé,
protégé et conçu pour les tensions et normes applicables. Valider le signal à
l’oscilloscope sur une charge basse tension appropriée.

## Protocole

La liaison USB série fonctionne à 115200 bauds. L’émetteur reçoit les commandes
`SET_BAUD:<débit>\n`, puis `SEND_TXT\n`, `SEND_IMGBW\n` ou `SEND_IMG256\n`.
Il répond respectivement `OK BAUD <débit>`, `READY`, puis `DONE` après émission.

Une trame CPL est composée de SOT (`0xF0`), du type (`0x8F` TXT, `0xF8` IMGBW,
`0x55` IMG256), des métadonnées little-endian, du payload et d’EOT (`0x0F`).
La longueur TXT occupe 2 octets. Les images transmettent les lignes et les
colonnes sur 2 octets chacune. Le récepteur vérifie l’EOT, puis transmet au PC
un entête USB `RCV_TXT`, `RCV_IMGBW` ou `RCV_IMG256`, suivi du payload binaire et
du statut `RCV_OK`.
