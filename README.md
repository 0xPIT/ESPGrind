# ☕️ ESPGrind

*Experimental* Espresso Mill Controller
(c)2025 karl@pitrich.com
License: MIT

### Overview

This is a touchscreen controller for an espresso mill that runs on an ESP32 with touchscreen.

I made this because I got a very good espresso grinder with a broken controller super cheap. As always, the spare parts are unobtainuim or super expensive. (#rightToRepairNow)

Beware: This was developed on a car ride from Munich to Berlin, so it will be mostly buggy.
I had a ESP32-BOX, so this is what is runs on. But it should be trivial to run on any other ESP32 baords with a touchscreen that works with [LVGL](https://docs.lvgl.io/master/intro/introduction.html) I used [Squareline Studio](https://squareline.io/) to create the UI quickly.

UI: I have another mill by a well-known Italian company called 🚀, and I enjoy their touch UI on my "👊o" mill daily, so I mostly ~~stole~~ borrowed it. I added nice arcs around the buttons that indicate the remaining time.

### Features
- 3 Programmable Timers (I, II, III)
- 1 Manual Button, creatively labeled M
- Simple screen for settings and showing counters
- Controls one relay via GPIO (I recommend to use a solid state relay)
- Settings, state and counters are automatically persisted in ESP32 NVM a few seconds after a change

### Hardware implementation
Use ESB-BOX, M5Stack, or similar ESP32 board with *capacitive* touchscreen (check for LVGL support)
Get a Mean Well IRM 03 AC Power Supply Module to power it, and a solid state relay (pcbs with 4 are cheap on $AMZN or $EBAY) to control the mill.

### Screenshots
![Main Screen](doc/screenshots/main.png)

![Mode Selected](doc/screenshots/selected.png)

![Running Timer](doc/screenshots/running.png)

![Settings Screen](doc/screenshots/settings.png)

