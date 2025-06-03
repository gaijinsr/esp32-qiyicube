# esp32-qiyicube
Interface for QiYi BLE smartcubes
![image](https://github.com/user-attachments/assets/59db5d69-81cc-49ce-9e38-6fd7a83e5b4f)
With the help of the great protocol analysis in https://github.com/Flying-Toast/qiyi_smartcube_protocol and borrowing from the structure of https://github.com/playfultechnology/esp32-smartcube,
I created code that can connect to a QiYi smartcube and output information about the current state of the cube, the last move made and the battery level via the serial port. The status of the
cube is also shown on a string of WS2812 LEDs that are wired in the standard order for cube states (URFDLB, zigzag from top left on each face).
