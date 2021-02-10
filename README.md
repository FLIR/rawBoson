FLIR Systems, Inc <BR>
2018 Nov <BR>

## Description

RawBoson is a tool that implements Boson FSLP protocol over serial.
It allows to send any command that is in Boson-ICD to the camera.
This package doesn't contain the Boson-ICD, in order to get one please refer to :

[FLIR Boson](http://www.flir.com/cores/boson/)

[ICD](https://www.flir.com/globalassets/imported-assets/document/boson_sdk_documentation_9hz-v11412-and-later.pdf)

It gets a function and its parameters, converts that to FSLP protocol and sends it through serial.

For example:
* send a FLAT Field:  ./rawBoson c50007
* ask for serial number : ./rawBoson c50002
* Change output to BlackHot : ./rawBoson cB0003 x0 x0 x0 x1
* Change output to WhiteHot : ./rawBoson cB0003 x0 x0 x0 x0

Other parameters:
* Use 'v' to see packets bytes: ./rawBoson c50007 v
* '-p' and '-b' configures serial port


## Instructions 
```
To compile : make
To clean   : make clean
```
## Add right permissions to serial port
sudo chmod a+rwx /dev/ttyACM0
