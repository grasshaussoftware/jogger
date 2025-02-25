# jogger

# The Jogger

## Introduction
"The Jogger" is a compact, standalone audio device designed for meeting tables. It continuously records the last 2 minutes of conversation in a rolling buffer and replays the most recent 15 seconds with a single button press. Built for privacy, it operates fully offline with no internet, Bluetooth, or Wi-Fi connectivity. This version uses the STM32F407 Discovery board for prototyping, with a 12-second buffer due to RAM limits (future versions will expand to 2 minutes).

Perfect for recalling that "what was just said?" moment in meetings!

## Features
- **Rolling Audio Buffer**: Stores up to 12 seconds (prototype) or 2 minutes (future) of audio.
- **Instant Playback**: Press a button to hear the last 15 seconds.
- **Offline Privacy**: No wireless connectivity.
- **Portable**: USB or battery-powered, fits on any table.
- **Simple Design**: Minimal components for easy assembly.

## Parts List
### Bill of Materials (BOM)
| Component                | Description                          | Quantity | Notes                              |
|--------------------------|--------------------------------------|----------|------------------------------------|
| STM32F407 Discovery      | Microcontroller development board    | 1        | Core of the device                |
| MAX9814 Microphone       | Electret mic with built-in amplifier | 1        | Or similar electret mic module    |
| 8Ω Speaker               | 0.5W mini speaker                    | 1        | Small enough for enclosure        |
| Push Button              | 6mm x 6mm momentary tactile switch  | 1        | For playback trigger              |
| 100µF Capacitor          | Electrolytic, for speaker decoupling | 1        | Blocks DC from speaker            |
| 0.1µF Capacitor          | Ceramic, for power smoothing         | 1        | Optional, near mic VCC            |
| 10kΩ Resistor            | Pull-up for button (if needed)       | 1        | Use if internal pull-up disabled  |
| Jumper Wires             | 24-26 AWG stranded wire              | ~10      | For connections                  |
| USB Cable                | Micro-USB for power/programming      | 1        | Supplies 5V to STM32             |
| Optional: LiPo Battery   | 3.7V with TP4056 charger module      | 1        | Alternative to USB power         |
| Optional: Enclosure      | Plastic box (100mm x 60mm x 25mm)    | 1        | For final assembly               |

### Tools Required
- Soldering iron and solder
- Wire cutters/strippers
- Small Phillips screwdriver
- Computer with Arduino IDE (STM32 core installed)
- Multimeter (optional, for troubleshooting)

## Assembly Instructions
### Step-by-Step Guide
1. **Prepare the STM32F407 Discovery Board**  
   - Unbox the board and connect it to your computer via USB. Confirm LEDs light up.
   - Install the STM32 Arduino core: Arduino IDE → Tools → Board Manager → Search “STM32 boards by STMicroelectronics” → Install.

2. **Connect the Microphone**  
   - Solder wires to the MAX9814 module: VCC (3.3V), GND, OUT.
   - Attach to STM32: VCC → 3.3V pin, GND → GND pin, OUT → PA0 (ADC1_IN0).
   - Secure with tape or a breadboard for now.

3. **Wire the Speaker**  
   - Solder a 100µF capacitor to the speaker’s positive leg (cap’s positive side to PB9).
   - Connect the speaker’s negative leg to GND.
   - Attach the capacitor’s free end to PB9 (TIM4_CH4 PWM).

4. **Add the Push Button**  
   - Solder wires to the button: one leg to PB0, the other to GND.
   - Optional: Add a 10kΩ resistor from PB0 to 3.3V if not using internal pull-up.

5. **Power Setup**  
   - Plug the USB cable into the STM32’s micro-USB port for 5V power (onboard 3.3V regulator used).
   - Optional: Connect a 3.7V LiPo to a TP4056 module, then 3.3V output to STM32 VIN, GND to GND.

6. **Flash the Firmware**  
   - Open Arduino IDE, select “STM32F407 Discovery” under Tools → Board.
   - Copy the code from the “Firmware” section below, upload via USB.
   - Open Serial Monitor (115200 baud) to confirm it’s running.

7. **Test It**  
   - Power on, speak near the mic, and press the button to hear playback.
   - Troubleshoot if needed: Check wiring, use Serial.print() for debugging.

8. **Enclosure (Optional)**  
   - Drill holes in a small box for the mic, speaker, and button.
   - Mount components with hot glue or screws; route USB out.

## Schematics
### Main Circuit Schematic
**Description**:  
This schematic shows the STM32F407 wired to the microphone, speaker, and button.  
- **STM32F407 Discovery**: Rectangular block with labeled pins: PA0, PB9, PB0, 3.3V, GND, 5V.
- **MAX9814 Microphone**:  
  - VCC → 3.3V (add 0.1µF cap to GND near VCC).
  - GND → GND.
  - OUT → PA0 (ADC input).
- **Speaker**:  
  - Positive → 100µF capacitor (+) → PB9 (PWM).
  - Negative → GND.
- **Push Button**:  
  - One leg → PB0 (GPIO).
  - Other leg → GND.
  - Optional: 10kΩ resistor from PB0 to 3.3V.
- **Connections**: Drawn as lines with labels (e.g., “PA0 to MIC_OUT”).

### Power Supply Schematic
**Description**:  
Shows USB and optional battery power options.  
- **USB Power**:  
  - Micro-USB connector → STM32F407 micro-USB port (5V in, onboard 3.3V out).
- **Battery Power**:  
  - 3.7V LiPo → TP4056 module (IN+, IN-).
  - TP4056 OUT+ → STM32 VIN (3.3V), OUT- → GND.
- **Connections**: Lines labeled “5V,” “3.3V,” “GND.”

## Firmware
### Code
```cpp
#include <Arduino.h>

#define PLAYBACK_PIN PB0
#define SPEAKER_PIN PB9
#define MIC_PIN PA0
#define SAMPLE_RATE 8000
#define BUFFER_SECONDS 12 // 12 sec for STM32F407’s 192KB RAM
#define BUFFER_SIZE (SAMPLE_RATE * BUFFER_SECONDS) // 96,000 samples

int16_t audioBuffer[BUFFER_SIZE];
volatile uint32_t bufferIndex = 0;

void setup() {
    pinMode(PLAYBACK_PIN, INPUT_PULLUP);
    pinMode(SPEAKER_PIN, OUTPUT);
    Serial.begin(115200);
    analogReadResolution(12); // 12-bit ADC
    Timer3.setPeriod(125); // 8kHz sampling (125µs)
    Timer3.attachInterrupt(recordAudio);
    Timer3.resume();
    Timer4.setPWM(SPEAKER_PIN, 16000, 0); // 16kHz PWM carrier
}

void loop() {
    if (digitalRead(PLAYBACK_PIN) == LOW) {
        delay(10); // Debounce
        if (digitalRead(PLAYBACK_PIN) == LOW) {
            playbackAudio();
        }
    }
}

void recordAudio() {
    int16_t sample = analogRead(MIC_PIN) - 2048; // Center around 0
    audioBuffer[bufferIndex] = sample;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

void playbackAudio() {
    uint32_t start = (bufferIndex - (SAMPLE_RATE * 15) + BUFFER_SIZE) % BUFFER_SIZE;
    for (uint32_t i = 0; i < SAMPLE_RATE * 15; i++) {
        int16_t sample = audioBuffer[(start + i) % BUFFER_SIZE];
        uint16_t pwmValue = (sample + 2048) >> 4; // Scale to 0-255
        analogWrite(SPEAKER_PIN, pwmValue);
        delayMicroseconds(125); // 8kHz playback
    }
    analogWrite(SPEAKER_PIN, 0); // Silence speaker
}
```
## Flashing Instructions
- Install Arduino IDE and the STM32 core via Tools → Board Manager → “STM32 boards by STMicroelectronics.”
- Connect the STM32F407 Discovery board to your computer via USB.
- In Arduino IDE, select “STM32F407 Discovery” under Tools → Board.
- Upload the code from the “Firmware” section; open Serial Monitor at 115200 baud to verify.

## Usage
- **Power On**: Connect via USB or switch on the battery.
- **Position**: Place on a table with the microphone facing speakers.
- **Record & Replay**: Speak near the mic, then press the button to replay the last 15 seconds.
- **Power Off**: Disconnect USB or turn off the battery.

## Future Enhancements
- Upgrade to STM32F746 for a full 2-minute buffer (320KB+ RAM).
- Design a custom PCB for smaller size and better power efficiency.
- Add a noise-canceling microphone and improved speaker for better audio quality.
- Create a 3D-printed enclosure for a sleek, professional look.

## Troubleshooting
- **No Playback**: Check connections: mic OUT to PA0, speaker to PB9, button to PB0.
- **Distorted Sound**: Ensure the 100µF capacitor is in series with the speaker; adjust mic gain on MAX9814 if applicable.
- **Button Issues**: Verify pull-up (internal or 10kΩ resistor); debug with `Serial.println(digitalRead(PB0));`.

## License
MIT License. Free to use, modify, and distribute.

## Contributing
Fork this repository, make improvements, and submit a pull request! Suggestions are welcome.

*Created by Brent Kaimanu Kohler on February 24, 2025.*
