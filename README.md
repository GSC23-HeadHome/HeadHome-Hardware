<br />
<div align="center">
<kbd>
<img src="https://firebasestorage.googleapis.com/v0/b/gsc23-12e94.appspot.com/o/members%2Fdaozheng.jpeg?alt=media&token=96a55b42-7c9f-4e68-b41f-d986efe79c01" height="50">
</kbd>
<kbd>
<img src="https://firebasestorage.googleapis.com/v0/b/gsc23-12e94.appspot.com/o/members%2Fhuixiang.jpeg?alt=media&token=72cf45a5-8208-46e1-9130-34b23755c574" style="border-radius:50%" height="50">
</kbd>
<kbd>
<img src="https://firebasestorage.googleapis.com/v0/b/gsc23-12e94.appspot.com/o/members%2Fjingxuan.jpeg?alt=media&token=0fb3ca79-d011-4ca9-8385-6720ac5a0f5d" style="border-radius:50%" height="50">
</kbd>
<kbd>
<img src="https://firebasestorage.googleapis.com/v0/b/gsc23-12e94.appspot.com/o/members%2Fmarc.jpeg?alt=media&token=491e54a5-3ff3-4204-b632-e01ed7317bd0" style="border-radius:50%" height="50">
</kbd>
</div>
<br />
<div align="center">
<img src="https://img.shields.io/badge/Built%20By-HeadHome%20Team-blue?style=for-the-badge">
</div>
<br />
<h1 align="center"> HeadHome Hardware </h1>

The **HeadHome wearable** is responsible for 
enabling the dementia patient to seek for help and providing directions when they are lost.

![HeadHome](assets/HeadHome.png)

<h2 align="center">👨🏻‍💻 Technology Stack</h2>
<br />
<div align="center">
  <img src="https://brandslogos.com/wp-content/uploads/images/large/arduino-logo-1.png" height="50" />
  <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/1/18/ISO_C%2B%2B_Logo.svg/1200px-ISO_C%2B%2B_Logo.svg.png" height="50">
  <img src="./assets/espressif-systems-seeklogo.com.svg" height="50" />
  <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/a/a0/ArduinoJson_logo.svg/2560px-ArduinoJson_logo.svg.png" height="50" />
  <h4>Arduino | C++ | ESP32 | ArduinoJson</h4>
</div>
<br />

<h2 align="center"> 🛠️ &nbsp;Key Functionalities </h2>

![HeadHome Functionalities](assets/HeadHome-Functionalities.png)
## 1. Request for help

Whenever the dementia patient is lost, `pressing the red button` on the wearable sends a notification to the Caregiver, alerting the Caregiver of their precarious situation.

## 2. Directions to Home

In addition, upon `pressing the red button`, the wearable displays an optimal path to guide the patient back home.

## ⚙️ &nbsp;Steps to Setup
1. Install the Arduino IDE from the link [here](https://www.arduino.cc/en/software).
2. Clone this repository and open [headhome-hardware.ino](headhome-hardware.ino) in the Arduino IDE
3. Upload the code onto your ESP32 and the code will run!

## 🔑 &nbsp;Key Files and Directories

- `assets`: Folder for all image assets
- `utils`: Folder for all helper utilities
- `headhome-hardware.ino`: Arduino ESP32 Hardware Code

## 🔗 &nbsp;Useful External Links
1. Image to arduino bitmap converter [link](https://javl.github.io/image2cpp/)
2. EEPROM interface [link](https://roboticsbackend.com/arduino-store-int-into-eeprom/)
