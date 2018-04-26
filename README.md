# SBSMark3
Third iteration of the SegbotSTEP family of robot designs. 

## Getting Started

Copy this repository into your Arduno development tree. Ensure that you have the ESP32
Arduino libraries installed.

### Prerequisites

This repository contains circuit diagrams created in EagleCAD,  

From a software perspective, this code base contains Arduino C++ code. You will need
an Arduino development environment and an ESP32 board to run this code. 

```
Note that this code will NOT run on an ESP8266. 
```

The following additional libraries are used:
* https://github.com/Pedroalbuquerque/ESP32WebServer
* https://github.com/Links2004/arduinoWebSockets
* https://github.com/marcoschwartz/LiquidCrystal_I2C
* https://github.com/bblanchon/ArduinoJson

From a hardware perspective, you will need to build an SBSMark3 compatible chassis. This
repository has EagleCAD diagrams to help you build the PCB. Also included are Fusion360 
CAD diagrams and associated GCODE for a Tormach 770 CNC mill to create the robot chassis. 
Full details for this project are located on the wiki associated with this repository.


### Installing

There are numerous tutorials on how to install A step by step series of examples that tell 
you have to get a development environment. See deployment section below for an overview
of the build environment used to create the repository. 


## Running the tests

At this time there are no automated tests for this system

### Break down into end to end tests

```
May create tests to help with PID tuning at some point.

```

## Deployment

This Arduino code was developed using the following set up.

## Built With

* [Visual Studio Code](https://code.visualstudio.com/) - The IDE
* [Platformio](https://platformio.org/) - Editor plugin
* Library Framework - Arduino
* Hardware platform - espressif32
* Board - feather32 

## Contributing

This project is being developed by a couple of buddies 

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/va3wam/SBSMark3/branches/all). 

## Authors

* ** Doug ** - *PID and motor control logic*
* ** Andrew ** - *Initial work* 

See also the list of [contributors](https://github.com/va3wam/SBSMark3/graphs/contributors) who participated in this project.

## License

This project has no licensing terms. Use at own risk, no support provided. 

## Acknowledgments

* Big thanks to [Brokking](http://www.brokking.net/yabr_main.html) for his excellent tutorial which really helped us get going
* Thanks to the folks who developed the libraries we have made use of for I2C, JSON, Web and Websockets

