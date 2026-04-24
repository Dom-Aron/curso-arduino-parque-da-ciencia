# Arduino Course — Parque da Ciência Newton Freire Maia

<p align="center">
  <strong>Teacher training in experimental Physics activities with Arduino</strong>
</p>

<p align="center">
  <a href="README.md">Versão em português</a>
</p>

<p align="center">
  <a href="https://github.com/Parque-da-Ciencia/curso-arduino-parque-da-ciencia">
    <img src="https://img.shields.io/badge/GitHub-Parque--da--Ci%C3%AAncia-181717?style=for-the-badge&logo=github" alt="Parque da Ciência GitHub">
  </a>
  <img src="https://img.shields.io/badge/Arduino-Experimentation-00979D?style=for-the-badge&logo=arduino&logoColor=white" alt="Arduino">
  <img src="https://img.shields.io/badge/Physics-Experimental%20Teaching-blue?style=for-the-badge" alt="Experimental Physics">
</p>

---

## About the project

This repository contains the code, support materials, and examples used in the teacher training course offered by **Parque da Ciência Newton Freire Maia**, focused on using the **Arduino** platform in experimental Physics activities.

The course aims to bring Basic Education teachers closer to accessible practices in instrumentation, data acquisition, automation, and visualization of physical phenomena, using simple electronic components and C/C++ programming for Arduino.

The material was organized to support the training sessions and to serve as a later reference for lesson planning, workshops, experimental demonstrations, and school projects.

---

## Goals

- Introduce Arduino as a tool for experimental teaching.
- Connect programming, basic electronics, and Physics concepts.
- Develop practical activities with sensors, actuators, and displays.
- Encourage the construction of low-cost experiments for classroom use.
- Support teachers in creating, adapting, and documenting inquiry-based activities.

---

## Repository structure

The repository is organized into two main versions of the code:

```text
curso-arduino-parque-da-ciencia/
├── ArduinoIDE/          # Code prepared for direct use in the Arduino IDE
├── VSCode/              # Code prepared for VS Code / PlatformIO or similar workflows
├── docs/                # Documentation, auxiliary materials, and LaTeX files
├── tools/               # Utility scripts for images, documentation, and processing
└── README.md
```

The separation between **ArduinoIDE** and **VSCode** allows the same content to be used in different development environments, according to participants' profiles and familiarity.

---

## Lessons and experiments

| Lesson | Folder | Topic | Concepts covered |
|---|---|---|---|
| Lesson 01 | `aula-01-blinking_led-ArduinoIDE` | Blinking LED | Digital outputs, timing, and the basic structure of an Arduino sketch |
| Lesson 02 | `aula-02-potentiometer-ArduinoIDE` | Potentiometer | Analog inputs, ADC reading, and variable control |
| Lesson 03 | `aula-03-ohm_law` | Ohm's Law | Voltage, current, resistance, data acquisition, and graphs |
| Lesson 04 | `aula-04-thermodynamic_law-ArduinoIDE` | Thermodynamics without LCD | LM35 sensor, temperature, analog reading, and Serial Monitor/Plotter |
| Lesson 04 | `aula-04-thermodynamic_law+LCD-ArduinoIDE` | Thermodynamics with LCD | LM35 sensor, temperature, hysteresis, and LCD I2C display visualization |
| Lesson 05 | `aula-05-hall_effect_latch_sensor-ArduinoIDE` | Hall latch sensor | Magnetic field, Hall sensor, digital detection, and event counting |
| Lesson 06 | `aula-06-LM35-ArduinoIDE` | LM35 sensor | Temperature, simple calibration, hysteresis, and serial visualization |
| Lesson 07 | `aula-07-lcd_display-ArduinoIDE` | LCD display | I2C interface, text, custom characters, and animations |
| Lesson 08 | `aula-08-speed_of_sound-ArduinoIDE` | Speed of sound | Time measurements, distance, sound propagation, and experimental estimation |
| Lesson 09 | `aula-09-driver-ArduinoIDE` | Transistor and relay driver | Load switching, BC548 transistor, relay, protection diode, and base current |
| Lesson 10 | `aula-10-stopwatch-ArduinoIDE` | Stopwatch | Time measurements, buttons, LCD I2C, and state organization |
| Lesson 11 | `aula-11-gravitational_acceleration-ArduinoIDE` | Gravitational acceleration | Free fall, sensors, time measurements, and estimation of g |

---

## Components used

Throughout the lessons, components such as the following are used:

- Arduino Uno or compatible board;
- LEDs and resistors;
- potentiometer;
- LM35 temperature sensor;
- 16×2 LCD display with I2C module;
- push button;
- Hall sensor;
- infrared phototransistors;
- 5 V relay;
- BC548 transistor;
- 1N4007 diode;
- jumper wires and breadboard.

The exact list of components may vary depending on the lesson and on adaptations made by the teacher.

---

## How to use

### 1. Clone the repository

```bash
git clone https://github.com/Parque-da-Ciencia/curso-arduino-parque-da-ciencia.git
cd curso-arduino-parque-da-ciencia
```

### 2. Choose the environment

To use the material with the **Arduino IDE**, open:

```text
ArduinoIDE/
```

To use it with **VS Code** or another development workflow, open:

```text
VSCode/
```

### 3. Open the desired lesson

Each lesson has its own folder, containing the main code and, when necessary, auxiliary documentation, scripts, or data collection files.

### 4. Upload the code to the Arduino

Connect the Arduino board to the computer, select the correct board and port in the development environment, and upload the code.

---

## Software requirements

To follow the course, the following software is recommended:

| Software | Use in the course | Download |
|---|---|---|
| Arduino IDE | Open, compile, and upload code to the Arduino board | [arduino.cc/en/software](https://www.arduino.cc/en/software) |
| VS Code | Alternative environment for code editing and project organization | [code.visualstudio.com](https://code.visualstudio.com/) |
| Python 3 | Data collection, processing, and visualization in some lessons | [python.org/downloads](https://www.python.org/downloads/) |
| Git | Clone the repository and track project updates | [git-scm.com/downloads](https://git-scm.com/downloads) |

It may also be necessary to install the driver for the Arduino board being used, especially for compatible boards that use USB-serial converters such as CH340 or CP2102.

Some lessons use additional libraries, such as support for I2C LCD displays. When required, the expected library is indicated in the code header.

---

## Branch organization

This repository uses two main branches:

| Branch | Purpose |
|---|---|
| `main` | Stable and public version of the material |
| `develop` | Development version for preparing future updates |

The recommended workflow is to develop new lessons, corrections, and improvements in `develop`, and only merge into `main` when the material has been reviewed.

---

## Documentation standard

The course code follows a documentation standard in Portuguese, including:

- institutional header;
- experiment description;
- identification of expected components;
- explanatory comments in the code;
- Doxygen-style function documentation when applicable.

This approach aims to make the material more accessible to teachers, students, and contributors who want to study, adapt, or expand the activities.

---

## About Parque da Ciência Newton Freire Maia

**Parque da Ciência Newton Freire Maia**, located in Pinhais, Paraná, Brazil, is a space dedicated to science communication, education, and the connection between science, technology, culture, and society.

This repository is part of its actions related to teacher training, experimentation, and the production of educational materials associated with open technologies in Science and Physics teaching.

---

## Credits

| Category | Information |
|---|---|
| Institution | Parque da Ciência Newton Freire Maia |
| Course | Teacher Training — Experimental Physics Activities with Arduino |
| Original code author | Aron da Rocha Battistella |
| Author GitHub | [Dom-Aron](https://github.com/Dom-Aron) |
| GitHub organization | [Parque-da-Ciencia](https://github.com/Parque-da-Ciencia) |
| Repository | [curso-arduino-parque-da-ciencia](https://github.com/Parque-da-Ciencia/curso-arduino-parque-da-ciencia) |

---

## Contributions

Contributions are welcome, especially for:

- fixing errors;
- improving comments and documentation;
- adapting activities to different school contexts;
- adding new experiments;
- creating support materials;
- standardizing code and examples.

Before contributing, it is recommended to open an issue or discuss the proposal with the repository maintainers.

---

## License

This repository uses a separate licensing policy for source code and educational materials:

| Type of content | License |
|---|---|
| Source code, scripts, and programming examples | MIT License |
| Texts, guides, images, documentation, and educational materials | Creative Commons Attribution 4.0 International — CC BY 4.0 |

In summary: the code may be reused, modified, and distributed with attribution; the educational materials may be shared and adapted, provided that proper credit is given to Parque da Ciência Newton Freire Maia and the authors.

See the repository license files for the complete terms:

- [`LICENSE`](LICENSE) — general licensing policy;
- [`LICENSE-CODE`](LICENSE-CODE) — MIT License terms for code;
- [`LICENSE-DOCS`](LICENSE-DOCS) — CC BY 4.0 terms and reference for educational materials.

---

<p align="center">
  <strong>Parque da Ciência Newton Freire Maia</strong><br>
  Science, technology, and public education in action.
</p>
