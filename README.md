# NSI Student POD — Firmware & Sensor Examples

Open-source Arduino sketches for the **Near Space Investigation (NSI)** student experiment **PODs**.
Start from the blank template (or an example close to your sensor), add your sensor code, and your data
rides the balloon to the ground — relayed by the Flight Computer and shown live in **NSI Mission Control**.

> Developed by **Atlantis Educational Services, Inc.** for the Near Space Investigation® program.

---

## What is a POD?

A POD is a small Arduino experiment board (an **Arduino Uno** or **Mega 2560**) with a **2.4 GHz XBee shield**.
It rides on the balloon next to the **Flight Computer (FC)**. During flight the FC repeatedly *polls* each POD
by its **POD ID**, and the POD answers with up to **ten 32-bit floating-point values** — `val_01` … `val_10`.

Those ten values are added to the FC's telemetry stream, sent to the ground over the 900 MHz radio link, and
appear in **NSI Mission Control** and the saved flight CSV. Your job as a student is to read a sensor and put
its readings into `val_01` … `val_10`.

- Up to **3 PODs** can fly at once. Each must use a **different POD ID (1, 2, or 3)**.
- Unused channels are simply left at `0.0`.

---

## Quick start

1. **Pick a sketch** from the table below (start with the **Blank template**, or an example that matches your sensor).
2. **Set the POD ID.** Near the top of the sketch, change `#define PODID 1` to `1`, `2`, or `3` — a unique number for this pod.
3. **Install the libraries** the sketch needs (see **Libraries to install** below). The Analog, Random, and Geiger sketches need none.
4. **Remove the XBee shield to upload.** Most XBee shields block the USB serial port, so you *cannot* program the board with the shield on.
   Take the shield off, upload over USB, then put the shield back to run with the Flight Computer.
5. **Select your board** in the Arduino IDE (Tools → Board): *Arduino Uno* or *Arduino Mega or Mega 2560*.
6. Upload, re-attach the shield, and watch your channels appear in Mission Control.

> ⚠️ **Never use `delay()` (or other slow/blocking calls) in `loop()`.** The FC polls your POD constantly; if your
> code is stuck in a delay, it will miss the poll and drop out of the data stream. Every example here uses a **timer**
> (`millis()`) instead of `delay()`, and slow sensors (like the DS18B20) use a non-blocking "request now, read later" pattern.

---

## The sketches

| Sketch (folder) | Sensor / purpose | Libraries needed | Data channels |
|---|---|---|---|
| **`SpaceTrek_Academy_Blank_XBEE_SHIELD_v3`** | **Start here** — empty template; add your own sensor | none | you define |
| `SpaceTrek_Academy_Random_XBEE_SHIELD_v3` | **Test data** (no sensor) — confirm the pod → FC → Mission Control chain works | none | `val_01–10` = random test values |
| `SpaceTrek_Academy_POD_Analog_Sensors_v3` | Analog sensors: photoresistor (light) + TMP36 (temperature) | none | `01` LDR counts · `02` LDR volts · `03` TMP36 °C |
| `SpaceTrek_Academy_POD_DS18B20_Temperature_v3` | DS18B20 1-Wire digital temperature (non-blocking pattern) | OneWire, DallasTemperature | `01` °C · `02` °F |
| `SpaceTrek_Academy_POD_DHT22_TempHumidity_v3` | DHT22 (AM2302) temperature + humidity | DHT sensor library | `01` °C · `02` %RH · `03` heat index °C |
| `SpaceTrek_Academy_POD_BME280_Environment_I2C_v3` | BME280 I²C: temperature, humidity, pressure, altitude | Adafruit BME280 Library | `01` °C · `02` %RH · `03` hPa · `04` m |
| `SpaceTrek_Academy_POD_UV_VEML6075_I2C_v3` | VEML6075 I²C ultraviolet (same UV sensor family as the Flight Computer) | Adafruit VEML6075 Library | `01` UVA · `02` UVB · `03` UV index |
| `SpaceTrek_Academy_POD_MPU6050_Motion_I2C_v3` | MPU6050 I²C accelerometer + gyroscope + temperature | Adafruit MPU6050 | `01–03` accel X/Y/Z m/s² · `04–06` gyro X/Y/Z rad/s · `07` °C |
| `SpaceTrek_Academy_POD_MultiSensor_v3` | **Three sensor types at once**: DS18B20 + photoresistor + BME280 | OneWire, DallasTemperature, Adafruit BME280 Library | `01` DS18B20 °C · `02` light · `03` hPa · `04` %RH · `05` BME280 °C |
| `SpaceTrek_Academy_POD_GeigerCounter_v3.0_XBEE_SHIELD` | Interrupt pulse counting (Geiger tube, anemometer, reed switch…) | none | `01` counts / minute · `02` rolling average |

Every sketch builds for both the **Uno** and the **Mega 2560** — just select the matching board before uploading.
(On a Mega the I²C pins are **SDA = 20, SCL = 21** instead of the Uno's **A4 / A5**.)

---

## Libraries to install

Some sketches use a sensor library. **You install these on your own computer** — nothing is bundled here, so grab a
fresh copy from the Arduino Library Manager. In the Arduino IDE:

**Tools → Manage Libraries…** → type the exact name below → **Install**. If it offers to install dependencies too,
choose **Install All**.

| Search this exact name | By | Used by | Automatically also installs |
|---|---|---|---|
| `OneWire` | Paul Stoffregen | DS18B20, MultiSensor | — |
| `DallasTemperature` | Miles Burton | DS18B20, MultiSensor | *(uses OneWire — install that too)* |
| `DHT sensor library` | Adafruit | DHT22 | Adafruit Unified Sensor |
| `Adafruit BME280 Library` | Adafruit | BME280, MultiSensor | Adafruit Unified Sensor, Adafruit BusIO |
| `Adafruit VEML6075 Library` | Adafruit | UV (VEML6075) | Adafruit BusIO |
| `Adafruit MPU6050` | Adafruit | MPU6050 | Adafruit Unified Sensor, Adafruit BusIO |

The **Analog**, **Random**, **Geiger**, and **Blank** sketches need **no** external libraries.

**Prefer to install by hand?** Download the `.ZIP` from the project page and use *Sketch → Include Library → Add .ZIP Library*:

- OneWire — https://github.com/PaulStoffregen/OneWire
- DallasTemperature — https://github.com/milesburton/Arduino-Temperature-Control-Library
- DHT sensor library — https://github.com/adafruit/DHT-sensor-library
- Adafruit BME280 Library — https://github.com/adafruit/Adafruit_BME280_Library
- Adafruit VEML6075 Library — https://github.com/adafruit/Adafruit_VEML6075
- Adafruit MPU6050 — https://github.com/adafruit/Adafruit_MPU6050
- Adafruit Unified Sensor — https://github.com/adafruit/Adafruit_Sensor
- Adafruit BusIO — https://github.com/adafruit/Adafruit_BusIO

Each sketch also lists its required libraries (with these links) in the comment block at the top of the file.

---

## How the code works

Every sketch shares the same skeleton (from the Blank template):

```
#define PODID 1          // 1, 2 or 3 — unique for each pod

union u_float val_01 ... val_10;   // the ten values you send to the Flight Computer

void setup()  { Serial.begin(57600); /* start your sensor */ }

void loop() {
  if ((millis() - timer) >= TIMER_TIME) {   // a timer — NOT delay()
    timer = millis();
    // read your sensor and store floats:
    val_01.value = ...;
    val_02.value = ...;
  }

  // ===== XBEE FC Communication =====  <-- DO NOT CHANGE
  // (this is how the Flight Computer polls your pod and reads your ten values)
}
```

**You edit only these regions** (they are clearly marked in every file):

- the **header comment**,
- the **library includes**,
- your **`#define` settings** and **sensor objects**,
- **`setup()`** (pin modes, `sensor.begin()`),
- the **timer block** inside `loop()` (read the sensor → assign `val_01 … val_10`),
- any **helper functions** (place them *above* the Binary Data Functions).

**Never edit** the four protected sections — the *Special Bytes*, the *FC Communication variables*, the
*XBEE FC Communication* block in `loop()`, and the *Binary Data Functions*. That code is the protocol the Flight
Computer uses to poll your pod and decode your values; it is byte-for-byte identical in every sketch here, so you can
always compare your file to the template to see exactly what you added.

### Adding your own sensor (from the Blank template)

1. `#include` your sensor's library and create its object.
2. In `setup()`, call the sensor's `begin()` (and any `pinMode()`).
3. In the **timer block**, read the sensor and store each reading as a **float** in `val_01`, `val_02`, … Cast integers with `float(x)`.
4. Note which channel holds what (and its units) in the header comment — those are the numbers you'll read in Mission Control.
5. Keep it non-blocking: no `delay()`; if a sensor is slow, request the reading one cycle and read it on a later cycle (see the DS18B20 example).

---

## Wiring quick reference

- **I²C sensors** (BME280, VEML6075, MPU6050): `SDA → A4`, `SCL → A5` on the Uno (`20` / `21` on the Mega). Power to 3.3–5 V, common GND. Most breakout boards already include the I²C pull-up resistors.
- **DS18B20 (1-Wire)**: DATA → D2, with a **4.7 kΩ pull-up** from DATA to 5 V (required, or it reads −127).
- **DHT22**: DATA → D2, with a **10 kΩ pull-up** to 5 V (many breakout boards include it).
- **Analog** (photoresistor / TMP36): sensor output → an analog pin (A0, A1, …).
- **Geiger / pulse source**: pulse output → **D2** (a hardware-interrupt pin), common GND.

---

## Credits & license

This code is published under the **[NSI POD Firmware License](LICENSE.md)** — in short: free to use and
modify with genuine NSI hardware for the NSI program; no redistribution outside your organization, no
commercial use, and no use to build cloned or competing hardware.

Firmware and examples © 2026 **Atlantis Educational Services, Inc.**, for the **Near Space Investigation®** program.
Base template and the Random / Geiger examples by Andrew Gafford. Sensor examples build on that template.

Sensor libraries referenced above are the property of their respective authors (Paul Stoffregen, Miles Burton, and
Adafruit Industries) and are installed separately by the user under their own licenses.
