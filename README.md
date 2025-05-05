This project presents an automated room temperature and lighting control system designed to enhance energy efficiency and comfort. The system uses PID (Proportional-Integral-Derivative) control to regulate room temperature by adjusting the speed of fans—either DC or AC motors. DC fans are controlled through PWM signals using an L298N motor driver, while AC fans are managed via a triac-based phase control circuit.

To manage lighting, an LDR sensor measures ambient light and adjusts the internal lighting accordingly. Additionally, two infrared (IR) sensors act as a bi-directional counter to track the number of people entering and leaving the room. The system activates only when at least one person is detected, reducing unnecessary energy usage.

Users can set their desired temperature using two buttons—one for increasing and one for decreasing the target temperature. The temperature control system engages only when the room temperature is below the set value, ensuring energy efficiency.

Overall, this system intelligently manages room temperature and lighting, promoting automation and sustainability in indoor environments.
