# The Ultimate Hobby Project

This code is released under the MIT licence.

### Project Description

This is the firmware for a self-balancing two-wheeled robot.
The robot uses stepper motors for transport, an LQG controller for stability, and a Kalman filter for sensor filtering.
This project has been a great way for me to develop practical skills in many different areas.  If you would like to see a technical write-up of this project, you can find it [here](https://drive.google.com/file/d/1yd39i1YjvD11JdsaPKPIDzlGQhWOL76k/view?usp=drive_link).

I hope that this repo will encourage you to try to conquer the inverted pendulum as I have. :)

### Requirements

This project utilizes a [Nano 33 BLE](https://docs.arduino.cc/hardware/nano-33-ble) as its processor core and the [A4988](https://www.digikey.com/en/htmldatasheets/production/693406/0/0/1/a4988) motor driver to drive the [stepper motors](https://www.amazon.com/Usongshine-Nema17-Stepper-17HS4401S-Printer/dp/B0787BQ4WH/ref=asc_df_B07KW6B3ZX?tag=bingshoppinga-20&linkCode=df0&hvadid=80333123589250&hvnetw=o&hvqmt=e&hvbmt=be&hvdev=c&hvlocint=&hvlocphy=&hvtargid=pla-4583932701264073&th=1).  For power, it uses a [TalentCell YB1203000](https://www.talentcell.com/lithium-ion-battery/12v/yb1203000-usb.html) Lithium Ion battery.

On the software side, the code requires two arduino libraries: <BasicLinearAlgibra.h> and <Arduino_LSM9DS1.h>.  I couldn't find a good way to calculate the controller gain on the arduino board, so I used the Scipy and Numpy libraries in Python to [calculate](./Code/findControlGain.py) the controller and copied the values into the [arduino code](./Code/robotCode.ino).

Enjoy!!!
