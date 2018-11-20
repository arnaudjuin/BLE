# Bluetooth Low Energy project specification
___
## Introduction
This project objective is to implement a BLE device.
It has all the basic initialization stuff to run this code on the STM32F401RE Nucleo board associated with BlueNRG (IDB04A1 or IDB05A1) and MEMS Sensors extension board (IKS01A1 or IKS01A2).

___
## Description
This project is a basis made from what had been presented in the previous sessions. 

An APP/Ble module is already present. It contains an already complet ble_common.c file and an almost complet ble_service.c file.

-> Most of the code you'll need write for this project should be done in the "Ble" folder or in the main.c file
-> You'll have to implement the BLE logic (services/characteristics declarations + management) in ble_service.c/ble_service.h and create your own files if needed (it could be more elegant).

___
## Objectives
This project should proposed 5 GATT services:
* A LED controler
* A button press counter
* A raw data access to the motion MEMS data
* A raw data access to the environmental sensors
* A motion detection service

___
## Ble specifications
The following section specifies the behavior expected for this device regarding those services. It defines its bluetooth configuration, the GATT services, characteristics and the behavior awaited accessing those characteristics.

### Device identification
**Device name**
The device name is used to represent a device after connection. It is accessible through the device name characteristic of the GAP service. Its size has to be less than 8 bytes. (BlueNRG limitation)

*Its format has to be as follows : teamXX with XX your team number in decimal.*

**Local name**
The local name or broadcast name is used in advertising packets to represent the device.

*Its format has to be as follows : PP_teamXX with X your team number in decimal.*

**Bluetooth Address (MAC address)**
Every bluetooth device has a unique MAC address. You will have to set the MAC address of your device. Its format has to be as follow : CC:45:4C:45:00:XX with XX your team number in hexadecimal (i.e. team 58 will have this MAC address, CC:45:4C:45:00:3A).

*Note : the MAC addresses are dealt with in Little endian in the Bluetooth stack and thus in the Bluenrg ACI. Pay attention to the way you see MAC addresses in the code and how you see it displayed by your smartphone or computer.*

**Device Connectivity**
To control and limit discoverability, the discoverability and connectability mode is controlled by the user button.
By default the device is in discoverable undirected connection mode. After a connection/disconnection cycle, the device go in non discoverable and connectable mode. A long press on the user button should sets for 120 seconds the device in general discoverable mode. After this time the device is set back to non discoverable and connectable mode.

### GATT Services and Characteristics
####LED control service
**Service description**
Description The LED service’s goal is to give the control of the LED.
UUID : afceb8e0-0377-11e0-5ac2-0002a5d5c51b

**Characteristics**
The LED ReadWrite characteristic
    Reading the value of this characteristic returns the state of the LED (on or off).
    Writing a value on this characteristic changes the state of the LED (on or off).
    Notify subscribed devices each time the value of the led.
UUID         a4c3ffc1-aa17-11e0-5aba-0002a5d5c51b
Properties   Read/Write/Notify
Value size   1 byte
Value format 0 represents off state
Value format 1 represents on state

The LED Blink characteristic
    Writing a value on this characteristic to put the LED in a blink state.
UUID        
Properties   Write
Value size   4 bytes
Value format 2 last bytes represents the blink interval in ms
Value format 2 first bytes represents the blink duration in ms

####Counter service
**Service description**
Description The counter service’s goal it to give access to a counter value. On start of the firmware, the value of the counter is set to 0.Each time we press the User button, the value of the counter is incremented.
UUID : c609b3c3-09f8-11e7-539a-0002a5d5c51b


**Characteristic**
The counter characteristic
    Reading the value of this characteristic returns the current value of this counter.
    Notify subscribed devices each time the value of the counter changes.
UUID            9ab6739c-0065-11e7-11d7-0002a5d5c51b
Properties      Read/Notify
Value size      2 bytes
Value format    Unsigned little-endian

___
## Ble Tools
To control your development, you're invited to use tools such as :
* Android : [nRF Connect](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=fr)
* iOS : [nRF Connect](https://itunes.apple.com/fr/app/nrf-connect/id1054362403?mt=8)
___
## Debug
This project has a "Debug utils" module who allows us to print information into Console during debug using the DEBUG_PRINTF function. To enable this feature :
1. Build using "Debug" configuration
2. Right click on your project
3. Debug As > Debug Configurations...
4. Double click on "Ac6 STM32 Debugging"
5. Select the newly created debugging profile and select the startup tab
6. Scroll down to find the field "Run Commands" and add the following : monitor arm semihosting enable
7. Run the debug configuration

___
## Tips
RTFM, WTFM, Commit, Push

[Can be a good start](http://www.st.com/en/embedded-software/x-cube-ble1.html)
[Can be really useful](http://www.st.com/content/ccc/resource/technical/document/programming_manual/1c/7e/d3/a6/d0/52/4a/35/DM00141271.pdf/files/DM00141271.pdf/jcr:content/translations/en.DM00141271.pdf)
[Should be your best friend](http://lmgtfy.com/?q=bluenrg+stm32)
"# BLE" 
