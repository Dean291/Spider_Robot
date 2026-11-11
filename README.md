🕷️ Spider Robot - 4 Legs (STM32 & Arduino)
A 4-legged spider robot project powered by an STM32F103C8T6 (Blue Pill) microcontroller, a PCA9685 servo driver, and 12 MG90S micro servos.

The robot is programmed using Arduino IDE (via the STM32duino core), combining the processing power of the STM32 with the simplicity of the Arduino framework.

✨ Key Features (V25)
Smart Pop-up (Stand-up Routine): Smoothly folds and lifts the body from a resting position. It includes a weight compensation algorithm specifically tuned for the heavier rear section (rear legs).

Crawl Gait: A highly stable walking algorithm where 3 legs always remain on the ground. The stepping sequence is Front-Left -> Rear-Right -> Front-Right -> Rear-Left (1 -> 4 -> 2 -> 3). It features dynamic load-shifting (support) to prevent the body from tipping over during leg swings.

Wave Gesture: Before walking, the robot shifts its center of gravity to the diagonal leg, lifts the front-left leg, and waves to greet.

Obstacle Avoidance: Utilizes an HY-SRF05 ultrasonic sensor. When an obstacle is detected, the robot automatically performs a state machine sequence: Stop -> Backward -> Turn (to bypass the obstacle).

🛠️ Hardware Requirements
Microcontroller: STM32F103C8T6 (Blue Pill).

Servo Driver: PCA9685 (I2C interface).

Motors: 12x MG90S micro servos (or equivalent).

Sensor: 1x HY-SRF05 Ultrasonic Sensor (or HC-SR04).

Power Supply:

Dedicated 5V/5A UBEC (for PCA9685 and servos).

3.3V from ST-Link or USB (for the Blue Pill).

Chassis: Custom 3D printed frame.

🔌 Wiring & Pinout
I2C Communication (Blue Pill to PCA9685)
STM32 Blue Pill	PCA9685
PB6 (SCL)	SCL
PB7 (SDA)	SDA
3.3V	VCC
GND	GND
5V UBEC Output	V+ (Terminal)
Ultrasonic Sensor (HY-SRF05)
STM32 Blue Pill	HY-SRF05
PA3 (A3)	TRIG
PA2 (A2)	ECHO
5V	VCC
GND	GND
Servo Channel Mapping (PCA9685)
Note: The physical channels are mapped to match the actual left/right orientation of the robot.

A (FR - Front Right): Coxa (CH2), Femur (CH0), Tibia (CH1)

B (RR - Rear Right): Coxa (CH5), Femur (CH3), Tibia (CH4)

C (FL - Front Left): Coxa (CH8), Femur (CH6), Tibia (CH7)

D (RL - Rear Left): Coxa (CH11), Femur (CH9), Tibia (CH10)

💻 Development Environment
Open Arduino IDE (v2.x recommended).

Install the STM32 MCU based boards (STM32duino) via the Boards Manager.

Select Board: Generic STM32F1 series -> BluePill F103C8.

Upload method: OpenOCD STLink (SWD).

An external ST-Link V2 programmer is required to upload code (connect to SWDIO, SWCLK, GND, and 3.3V pins).

⚙️ Calibration & Tuning
Because every 3D printed chassis has slight mechanical variations, you may need to fine-tune the following parameters in the code:

Standard Stance (stand_pose): A 4x3 array containing the standard angles (Coxa, Femur, Tibia) for all 4 legs. Adjust these so the robot stands level with all feet touching the ground.

Rotation Direction (COXA_DIR, LIFT_DIR): If a specific leg steps in the wrong direction or points down into the ground instead of lifting, change the sign from 1 to -1 in the corresponding array.

Gait Parameters:

STRIDE_AMP: The stride length (coxa sweep amplitude). Reduce this if the robot stumbles.

LIFT_HEIGHT: The height the leg lifts off the ground. Reduce this if the servos are straining or buzzing.

CYCLE_MS: The duration of one full gait cycle (increase this if the robot moves too fast and becomes unstable).

📜 License
This project is shared for educational and research purposes. You are free to use, modify, and distribute it.
