# NSI Student POD — Baseline Firmware Releases

Known-good baseline binaries for the NSI student experiment PODs. Students normally customize the POD template source provided with the curriculum; flashing one of these binaries restores a pod to the working baseline (it responds to Flight Computer polls with test data).

Source code is proprietary to Atlantis Educational Services, Inc. and is not published here — this repository hosts official compiled firmware releases only.

## Which file?

Pick by your pod's board and its POD ID (1, 2 or 3 — each pod on a flight must use a different one):

- Arduino Uno-class board: `POD<id>_Uno_<version>.hex`
- Arduino Mega 2560 board: `POD<id>_Mega2560_<version>.hex`

Manual flash — Uno: `avrdude -patmega328p -carduino -P <port> -b115200 -D -Uflash:w:<hex>:i` · Mega: `avrdude -patmega2560 -cwiring -P <port> -b115200 -D -Uflash:w:<hex>:i`
