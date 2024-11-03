# MaTouch_SimpleOBD2
This example extends the MaTouch_SimpleCan example by adding OBD2 PID polling. Code is written to support
2 display sizes, the 2.1" and the 1.28" MaTouch round tft displays. This example is provided as-is. 
There are no guarantees this will work for your vehicle although it was tested on 2 different makes as well
as a generic DIY OBD2 ECU emulator.

Canbus and 12V adapter boards can be found here: 

https://www.tindie.com/products/cnjboards/can-bus-adapter-card-for-matouch-round-displays/




This code is supplied as a demonstration of:
1) cnjboards Canbus and 12V adapter card
2) Canbus with filtering on ESP32-S3 processor
3) Reading generic OBD2 using the Canbus
4) Displaying OBD2 values on a round tft using LVGL.
5) Simple rotary encoder mechanism for user input.

A simple OBD2 test rig for test use in a vehicle.

<img src="https://github.com/user-attachments/assets/91524e45-1c55-4a37-ad5d-fb50baa0ab1c" width="450" height="375"><img src="https://github.com/user-attachments/assets/2a2450bc-d08d-4633-97db-7269ee557f14" width="450" height="375">
<img src="https://github.com/user-attachments/assets/c1208390-5f0e-4a8d-8dbe-8328af4df076" width="450" height="375"><img src="https://github.com/user-attachments/assets/4c173855-c069-4c0c-ae36-c5be93d9750c" width="450" height="375">

Note: 2.1" display support is still under construction :-)
