# CAN Debug Guide - AGX Orin <-> CAN Bus <-> ESP32
This guide is to help with debugging the robotic arm's CAN - system.
The goal is not primarly to move or controll the arm, but to confirm the communication between the AGX ORIN <-> CAN-Buss <-> ESP32-nodes is stable and free from inormalitise.

## 0. Safety first
During CAN-debugging:
- Do NOT connect the motors if there is a risk that they can start moving.
- Start the debugging with the arm turned off.
- Power the ESP32 seperat with an USB-C from the computer.
- Do NOT send actuation-messages under the first debugging stages.
- Start with ONE ESP32-node at a time.

Our goal:
1. Start `can0` on Orin.
2. See CAN-trafic.
3. Control CAN-state and error counters.
4. Be able to tell if an ESP32/transceiver/cabel causes problem.

## 1. The chain

AGX Orin
   |
SocketCAN / can0
   |
CAN controller
   |
CAN transceiver
   |
CAN-H / CAN-L
   |
CAN transceiver
   |
ESP32

When the full arm is used:

                     +--> ESP32 shoulder
                     |
AGX Orin --> CAN BUS +--> ESP32 elbow
                     |
                     +--> ESP32 wrist
                     |
                     +--> ESP32 hand

The project uses a classic CAN 2.0 with standard 11-bit CAN IDs and max 8 databites per CAN-frame.

## 2. Important Files

### `agx_setup_can.sh`
Detta script:

- kontrollerar CAN-controller på Orin
- konfigurerar CAN-pinnar
- laddar Linux CAN-drivrutiner
- skapar `can0` och `can1`
- sätter bitrate
- aktiverar error reporting
- aktiverar automatisk Bus-Off recovery

Normalinställningen i filen är:

bash
ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100


Det betyder:

- bitrate = 1 Mbit/s
- CAN error reporting = på
- om Orin går Bus-Off försöker den återstarta CAN efter 100 ms

### `can_socketcan.py`
Python-wrapper runt SocketCAN. Den används senare för att läsa/skicka CAN-meddelanden från Python.

Använd inte den som första felsökningsmetod. Första felsökningen görs med `ip` och `candump`.

### `can_message_parser.py`
Tolkar CAN ID + bytes till begriplig data.

OBS: Dokumentationen och parsern verkar inte vara helt synkroniserade för vissa meddelanden, framför allt IMU och pressure. Kontrollera därför ESP32-koden innan parsern används som facit.

### `test_socketcan.py`
Kör inte denna fil först. Den innehåller testmeddelanden som inte helt matchar robotens verkliga CAN-protokoll.

## 3. Control physical CAN-bus
### 3.1 Turn of the power
All electronic on the CAN-bus should be turned of when the resistans is measured.

### 3.2 Measure Resistans between CAN-H and CAN-L

Use a multimeter for example to measure the resistance between CAN-H/L
Expected value with two 120 Ohm terminals: 60 Ohm

Interpretation:
~60 Ohm     = normal
~120 Ohm    = probably only one terminal
~40 Ohm     = probably three terminal
OL/high     = terminal missing or distrubtion


## 4. Connect ONE ESP32
First test setup:
Connect your computer with one ESP32 via an USB cabel so that the microcontroller has power and we can open a serial monitor in later steps. Your setup chain should look like this:

Laptop
  |
USB-C
  |
ESP32
  |
CAN transceiver
  |
CAN-H =================== CAN-H
CAN-L =================== CAN-L
  |
CAN transceiver
  |
AGX Orin

*Make sure the motors is not active.


## 5. Start CAN on the Orin
If you have not used the AGX Orin before you need to do some setup first. You could either:
  - Connect a screen,mouseand keyboard directly connected to the AGX ORIN, or 
  - Connect from a laptop.

For this setup guide the laptop method is used with VS Code and the Remote - SSH extension.

### SETUP:

#### Step 1: Install VS CODE and Remote - SSH.
   Install:
   - Visual Studio Code
   - The VS Code extension 'Remote - SSH'

#### Step 2: Start the Orin
   1. Connect power supply to the AGX Orin 
   2. Connect the laptop to the AGX Orin using USB-C port used for device mode/SSH.
   3. Start the AGX Orin
   4. Wait until the system has finished booting.
   Hardware layout: https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/latest/hardware_layout.html

#### Step 3: Verify the USB Connection
   Expected adresses are:
   Orin address: 192.168.55.1
   Laptop:       192.168.55.100

   To see that the AGX Orin is correctly connected and working open windows powershell and run:
      ping 192.168.55.1
   You should receive:
     Reply from 192.168.55.1: bytes=32 time<1ms TTL=64
   This means the Jetson is turned on and the USB connection is working correctly.
   If you get `request time out` or similar then the USB/startup/power need to be troubleshoot. 

#### Step 4: Test the SSH Connection
   Open VS Code/powershell  terminal and run: 
   ```bash
    ssh bigboyorin@192.168.55.1
   ```
   The first time you connect a message simliar to this may appear: 
      Are you sure you want to continue connecting?
   Write in the terminal "yes"
   Then write in the password(written on the Orin harware)

#### Step 5: Connect using VS Code
   Open VS Code and press:
   "Ctrl + Shift + P" and search for "Remote-SSH: Connect to Host..." and write:
      bigboyorin@192.168.55.1
   A new window with direct connection to Orin will open.
   
#### Step 6: Verify the connection.
   In the new window(SSH connection) open a new terminal and run:
   ```bash
      whoami
      hostname
      uname -a
   ```
   The output should confirm that the terminal belong to the AGX Orin and not the Windows laptop.

#### Step 7: Find the project root
   Run:
   ```bash
      pwd
      ls
   ```
   In the setup tested here the project folder was:
   LIMB-HT25
   Enter the project folder:
   ```bash
      cd LIMB-HT25
      ls
   ```
   This map contains scripts, src, docs and README.md file. 

#### Step 8: Verify that CAN interface exists
Before starting the CAN interface verify that the CAN interface are available. Run the following code and you should see can0, can1:

```bash
ip link show
```

If can0 or can1 does not appear when running ip link show, load the CAN driver manually:
```bash
sudo modprobe mttcan
```
Then check again: 
```bash
ip link show
```

#### Step 9: Inspect the CAN setup script
Once both can0/1 are available run the CAN setup script from the project root directory:
```bash
cd scripts
ls
```
The folder should cotain:
agx_setup_can.sh
Before running the script inspect it:
```bash
   cat agx_setup_can.sh
```
#### Step 10: Run the CAN setup script
Inside the scripts folder run:
```bash
   sudo ./agx_setup_can.sh
```
Or if you are in the project folder root run:
```bash
   sudo scripts/agx_setup_can.sh
```
Then verify that the CAN interfaces are UP:
```bash
   ip link show
```
Expected result:
can0: <NOARP,UP,LOWER_UP,ECHO>
can1: <NOARP,UP,LOWER_UP,ECHO>
If not UP, troubleshoot the CAN setup before continuing.

## 6. Control CAN-state

Run:
```bash
ip -d -s link show can0
```
Look for:
state ERROR-ACTIVE
bitrate 1000000
berr-counter tx 0 rx 0

Also inspect:
bus-errors
error-warn
error-pass
bus-off

Preferred state during normal operation: `ERROR-ACTIVE`.
IF the interface is in ERROR-PASSIVE or BUS-OFF, there is a communication problem that should be investigated before continuing.  

## 7. Listen to CAN-trafic
Now that `can0/1` has been created and the CAN state has been checked, the next step is to check if the AGX Orin is actually receiving CAN messages.
First we have to verify that the can utils are installed:
```bash
   which candump
   which cansend
```
Expected output:
/usr/bin/candump
/usr/bin/cansend

If not run the following to install the can-utils:
```bash
   sudo apt update
   sudo apt install can-utils
```
### Test 1: Listen on can0
Open terminal 1:
```bash
candump -x can0
```
Stop candump with Ctrl + C

### Test 2: Send a known CAN Frame
Keep terminal 1 open and candump can0 running. Open Terminal 2 and run:

```bash
cansend can0 123#11223344
```
Terminal 1 should now received:
can0  123  [4]  11 22 33 44
This means:
can0        = CAN interface
123         = CAN identifier
[4]         = 4 data bytes
11 22 33 44 = payload

Now AGX Orin CAN interface is active and that CAN frames can be sent and obvserved through can0. 
Now check the counter again:

```bash
ip -d -s link show can0
```

## 8. Control error during CAN - trafic
In Terminal 1 RUN:
```bash
candump -x can0
```
In Terminal 2 RUN:
```bash
watch -n 1 'ip -d -s link show can0'
```
CAN-status updates now every sec. In Terminal 2 look for:
state ERROR-ACTIVE
berr-counter tx 0 rx 0
bus-errors 0
error-pass 0
bus-off 0

This is good. If berr-counter, bus-errors, error-warn/pass or bus-off should not increase weirdly.
TX and RX should increase as CAN frames is send back and forth. 

## 9. Setup ESP-IDF
The ESP32 nodes in this project use native ESP-IDF.
If ESP-IDF is already installed and working, skip to the next step.

### 9.1 Check if ESP-IDF is installed
In VS Code check if the **ESP-IDF Explorer** icon is available in the left sidebar.
You can also open Powershell and check the installed ESP-IDF versions using the Espressif Installation manager:
For example:
```powershell
& 'C:\Users\<USERNAME>\.espressif\eim_gui\eim-gui-windows-x64-v0.20.0.exe' list
```
The output should look similar to:
installed versions:
v5.5.5 selected [ok] [...]

For this code ESP-IDF 5.5.x is preferred and built on. Newer versions could introduce compatibility problems.

### 9.2 If ESP-IDF VS Code extension is not insalled
Go to extensions and install ESP-IDF extension by Espressif Systems. If opening the page causes issues or you have problem with it open Powershell and run:
```bash
code --install-extension espressif.esp-idf-extension
```
Then restart VS Code, press `CTRL + SHIFT + P` and search for:
`ESP-IDF: Open ESP-IDF Installation Manager`

This opens the ESP-IDF Installation Manager.
If the Installation Manager opens correctly, check if ESP-IDF v5.5.5 is already installed.
If v5.5.5 is not installed, install:
- ESP-IDF version: `v5.5.5`
- Target: `esp32c3`
- Required tools and prerequisites

If the Installation Manager does not open correctly and gets stuck on:
Running GUI...

Continue with step 9.3 and install ESP-IDF through PowerShell instead.

### 9.3 If ESP-IDF is not installed or the Installation Manager GUI does not work
Check available commands and then currently installed versions:
```powershell
& 'C:\Users\<USERNAME>\.espressif\eim_gui\eim-gui-windows-x64-v0.20.0.exe' --help
& 'C:\Users\<USERNAME>\.espressif\eim_gui\eim-gui-windows-x64-v0.20.0.exe' list
```
To install ESP-IDF v5.5.5 for ESP32-C3
```powershell
& 'C:\Users\<USERNAME>\.espressif\eim_gui\eim-gui-windows-x64-v0.20.0.exe' install `
  --idf-versions v5.5.5 `
  --target esp32c3 `
  --install-all-prerequisites true
```
After installation is complete check again:
```powershell
& 'C:\Users\<USERNAME>\.espressif\eim_gui\eim-gui-windows-x64-v0.20.0.exe' list
```

### 9.4 Open an ESP-IDF terminal
In the ESP-IDF Explorere select:
`Open ESP-IDF Terminal`
And verify the ESP-IDF environment

```bash
idf.py --version
```
You should receive the version and not `idf.py is not recognized`, then you are not in the activated ESP-IDF environment.
Once `idf.py --version` works, the ESP-IDF environment is activated.
Continue with step 9.5 to verify that the project target is set to ESP32-C3.

### 9.5 Make sure the ESP-IDF target is set to ESP32-C3
The ESP32 nodes in this project use the ESP32-C3 target.
Inside the ESP-IDF terminal, first check if the environment already has a target set:

```powershell
echo $env:IDF_TARGET
```
If the output is: esp32 remove the target
```powershell
Remove-Item Env:IDF_TARGET
```

Then check again:
```powershell
echo $env:IDF_TARGET
```

The output should now be empty, then you can set the new target to ESP32-C3:
```powershell
idf.py set-target esp32c3
```

Now the project is ready to be built, flashed and monitored.

## 10. (CURRENT)Configure and monitor the ESP32 node

Så nu vill vi fokusera på ACK/TX-vägen från hand-modulen, inte på programlogiken.
Jag skulle göra felsökningen i den här ordningen: först stäng av all ström, sedan mäta resistansen mellan CAN-H och CAN-L för att verifiera termineringen, och därefter kontrollera att hand-modulens transceiver verkligen är korrekt ansluten både till bussen och till ESP32:ns TX/RX. När den delen ser rätt ut startar vi upp systemet igen och kör exakt samma 261#00-test.

Orin använder just nu:

```text
restart-ms 100
```

Det gör att:

```text
CAN error
↓
Bus Off
↓
100 ms
↓
automatisk restart
↓
CAN försöker fungera igen
```

Det betyder att ett allvarligt problem kan döljas eftersom kommunikationen kommer tillbaka automatiskt.

Under felsökning:

- notera varje gång Bus-Off inträffar
- kontrollera hur ofta det händer
- ändra inte recovery-logiken direkt utan dokumentera först beteendet

ESP32-noderna kan också ha egen Bus-Off recovery via config.

Kontrollera för varje ESP32:

```text
Bus-Off recovery enabled? YES / NO
```

## 11. Steg 8 – Testa varje ESP32 separat

Testa:

```text
Orin + ESP32 shoulder
Orin + ESP32 elbow
Orin + ESP32 wrist
Orin + ESP32 hand
```

För varje test:

| Nod | ERROR-ACTIVE? | RX errors | TX errors | Bus-Off? | CAN-data syns? |
|---|---|---:|---:|---|---|
| Shoulder | | | | | |
| Elbow | | | | | |
| Wrist | | | | | |
| Hand | | | | | |

Om tre fungerar bra men en ger mycket errors är det en stark indikation på fel kring den noden:

- kabel
- connector
- PCB
- CAN-transceiver
- CAN-pins
- firmware/config

## 12. Steg 9 – Pressure module separat

Eftersom `robot_hand_pressure_module` tidigare haft mycket fel:

```text
AGX Orin
   |
CAN
   |
robot_hand_pressure_module

ESP32 powered from PC via USB-C
```

Kontrollera:

```bash
candump -x can0
```

och:

```bash
ip -d -s link show can0
```

Om pressure-modulen ensam fortfarande orsakar mycket errors, misstänk i första hand:

1. kabel / lös kontakt
2. fel CAN-H / CAN-L
3. jordproblem
4. CAN-transceiver
5. PCB-fel
6. fel GPIO/CAN-pin config
7. bitrate mismatch

## 13. Steg 10 – Bygg upp nätverket en nod i taget

När varje nod fungerar separat:

```text
Orin + ESP1
Orin + ESP1 + ESP2
Orin + ESP1 + ESP2 + ESP3
Orin + ESP1 + ESP2 + ESP3 + ESP4
```

Efter varje tillagd nod:

```bash
ip -d -s link show can0
```

och:

```bash
candump -x can0
```

Notera exakt när problemen börjar.

## 14. Steg 11 – Jämför robot power OFF och ON

Test A:

```text
Robot power OFF
ESP32 via USB
CAN connected
```

Test B:

```text
Robot electronics ON
Motors OFF
CAN connected
```

Test C:

```text
Robot electronics ON
Motors enabled
CAN connected
```

Fyll i:

| Test | RX errors | TX errors | Bus-Off? | Kommentar |
|---|---:|---:|---|---|
| Robot OFF | | | | |
| Electronics ON | | | | |
| Motors ON | | | | |

Om errors ökar tydligt när motorerna aktiveras, misstänk:

- EMI
- ground bounce
- spänningsdippar
- dålig jordning
- störningar på transceiver supply
- dålig CAN-kabel/topologi

## 15. Steg 12 – Testa lägre bitrate

Nuvarande bitrate:

```text
1 Mbit/s
```

Som diagnostiskt test kan ni prova:

```text
500 kbit/s
```

Alla noder måste använda samma bitrate:

```text
Orin      500k
ESP32 #1  500k
ESP32 #2  500k
ESP32 #3  500k
ESP32 #4  500k
```

På Orin behöver `agx_setup_can.sh` ändras från:

```bash
bitrate 1000000
```

till:

```bash
bitrate 500000
```

ESP32-koden måste ändras på motsvarande sätt.

Om:

```text
1 Mbit/s   = instabil
500 kbit/s = stabil
```

tyder det på problem med exempelvis:

- signalintegritet
- bus topology
- kabellängder
- terminering
- störningar
- timing

Se detta främst som ett diagnostiskt test.

## 16. Kontrollera jord och kablage

Kontrollera:

- [ ] CAN-H går till CAN-H
- [ ] CAN-L går till CAN-L
- [ ] alla connectors sitter ordentligt
- [ ] ingen kabel är lös
- [ ] gemensam GND finns där konstruktionen kräver det
- [ ] transceiverns supply är stabil
- [ ] inga ledningar går onödigt nära störande motorkablar
- [ ] terminering finns endast i bussens båda ändar

## 17. När rå CAN-kommunikation är stabil

Först när:

```text
candump fungerar
+
ERROR-ACTIVE stabilt
+
inga onormala error counters
+
ingen Bus-Off
```

går vi vidare till Python-programmen.

Ordning:

```text
1. agx_setup_can.sh
        ↓
2. can0
        ↓
3. candump
        ↓
4. stabil CAN-buss
        ↓
5. can_socketcan.py
        ↓
6. can_message_parser.py
        ↓
7. robot application
```

## 18. Använd inte motorstyrning som första test

Undvik actuation IDs under första testen:

```text
0x220
0x221
0x222
0x240
0x260
0x261 ...
```

De används för faktisk motorstyrning.

Första målet är att bara ta emot och observera CAN-trafik.

## 19. Dokumentationsproblem som ska utredas senare

### Pressure
`message_data_layout.md` beskriver pressure som 2-byte integer i millivolt, medan `can_message_parser.py` tolkar pressure som float32.

Det måste verifieras mot ESP32-koden.

### IMU
Dokumentationen beskriver att varje IMU-float skickas i separat CAN-frame eftersom klassisk CAN bara tillåter 8 data bytes.

Parsern verkar däremot förvänta sig tre float32 i samma meddelande för vissa IDs.

Tre float32 = 12 bytes, vilket inte får plats i klassisk CAN.

Därför måste aktuell ESP32-firmware användas som huvudsaklig referens.

## 20. Resultatmall

### Datum
`________________________`

### Setup

ESP32-nod:
`________________________`

Bitrate:
`________________________`

Robot power:
- [ ] OFF
- [ ] ON

Motorer:
- [ ] OFF
- [ ] ON

ESP32 powered by:
- [ ] Robot
- [ ] USB

### CAN status

```text
State:
TX error:
RX error:
Bus-Off:
```

### Candump output

```text

```

### ESP32 monitor output

```text

```

### Vad hände?

```text

```

### Nästa test

```text

```

## 21. Kort felsökningsordning

```text
1. Motorer OFF
2. Koppla EN ESP32
3. Mät CAN-H ↔ CAN-L ≈ 60 Ω
4. sudo scripts/agx_setup_can.sh
5. ip -d -s link show can0
6. candump -x can0
7. ESP32 serial monitor
8. Kontrollera errors / Bus-Off
9. Testa varje ESP separat
10. Bygg sedan upp nätverket en nod i taget
11. Testa robot power ON
12. Testa motors ON
13. Testa 500 kbit/s om nödvändigt
14. Först därefter Python och motorstyrning
```

## Frågor / anteckningar

Skriv gärna direkt under punkterna i den här filen när något är oklart.

Exempel:

```text
FRÅGA:
Vad menas med ERROR-PASSIVE?

SVAR:
[lägg in svar här]
```

På så sätt kan filen växa till projektets egen CAN-felsökningsmanual.
