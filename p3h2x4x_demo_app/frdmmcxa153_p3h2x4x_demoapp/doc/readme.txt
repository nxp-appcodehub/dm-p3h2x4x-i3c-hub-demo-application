Hardware requirements
===================
- Mini/micro C USB cable
- FRDM-MCXA153 board
- P3H2x4x I3C HUB (https://www.nxp.com/part/P3H2x4x-ARD#/)
- PCF2131 RTC DEVICE
- P3T1755TP Temperature sensor
- Personal Computer

Limitations

There are some limitation to the I3C hub which need to be kept in mind while using the application:
1. While reseting the hub device from the application, if anytime I3C bus hangs, perform power off reset and reconnect the usb port.
2. While testing the IBI feature, make sure to disable the hub network connection from the driver application( "I3C hub" folder->P3H2x4x_config.h). 
3. When hub device is in I2C mode, then only I2C and SMBUS target devices can function, not I3C devices.
4. When hub device is in I3C mode, then only I3C and SMBUS target devices can function, not I2C devices. 
5. Rework needs to be done in both MCXA153 and MCXN947 baseboard for I3C protocol.
  * MCXA153- external add on jumper on baseboard ie., J20 and J21.
  * MCXN947- Shorting of 2-3 pins of SJ14 and SJ15 
6.  To test I3C hub with FRDM-MCXA153, P3H2x4x need to be connected using jumper wires with the baseboard.
and with MCXN947, we can stack the shield board directly.
Sno.	P3H2x4x  		MCXN153
1	J5- pin 9 - SDA	        J20 
2	J5- pin 10 - SCL	J21
3	J4- pin 4 - VCC		J3 - pin 8
4	J4- pin 6 - GND		J3 - pin 12
5       J5- pin 1 - CP-RST	J2 - pin 2

Pre-requisites before testing the application

NOTE: In driver application ( "I3C hub" folder->P3H2x4x_config.h), default settings using macros have been done for testing purposes ie.,
Target port(TP 0) - On die temperature sensor in SMBUS mode
1. TP 2 - On die temperature sensor in I3C mode
2. TP 3 - External I3C device(P3T1755TP)
Sno. 	P3H2x4x (For TP-3)	P3T1755TP(For I3C)
1	J26 - pin 1- SCL	J13 - pin 1- SCL
2	J26 - pin 2- SDA	J13- pin 2- SDA
3	J26 - pin 4- GND	J13 - pin 4- GND
4	J26 - pin 3- VCC	J4 - pin 4 - VCC
				short 1-2 of JP1 &JP2

3. TP 6 - External I2C device(PCF2131)
Sno. 	P3H2x4x (For TP-6)	PCF2131(For I2C )
1	J29 - pin 1- SCL	J4 - pin 1- SCL
2	J29 - pin 2- SDA	J4 - pin 3- SDA
3	J29 - pin 4- GND	J4 - pin 2- GND
4	J29 - pin 3- VCC	J11 - pin 4 - VCC

4. TP 1 - External SMBUS device(P3T1755TP)
Sno.	P3H2x4x (For TP-1)	P3T1755TP(For Smbus)
1	J23 - pin 1- SCL	J5 - pin 10- SCL
2	J23 - pin 2- SDA	J5- pin 9- SDA
3	J23 - pin 4- GND	J4 - pin 6- GND
4	J23 - pin 3- VCC	J4 - pin 4 - VCC
				short 2-3  of JP1 &JP2
5. 0 - External SMBUS device(RTC-PCF2131) with FRDM-MCXA153 baseboard for IBI
Sno. 	P3H2x4x (For TP-0)	PCF2131(For IBI )
1	J10 - pin 2- SCL	J4 - pin 1- SCL
2	J10 - pin 6- SDA	J4 - pin 3- SDA
3	J29 - pin 4- GND	J4 - pin 2- GND

6. user wants to change the default configurations, Open the flashed code, select I3C Hub folder, select "P3H2x4x_config.h" file and change the Macros as per choice.

7. To test IBI feature, Disable Hub network connections from (I3C hub folder->P3H2x4x_config.h file) and also enable the IBI for respective target port to which an external controller is connected.

8. To change any default config of any variants, change macros in "P3H2x4x_config.h" file.

9. make sure, J35 jumper pin is shorted and J20 AND J34 is open on P3H2X4X. 

Prepare the Demo
===============
1.  Connect a USB cable between the host PC and the OpenSDA USB port on the target board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
==============

Main Menu will look like this**
- On boot, user need to set controller - Hub network connection by selecting either I2C or I3C. 

- NOTE: If I3C is selected, User needs to assign the dynamic address explicitly. 


 P3H2840 I3C HUB Initialization completed

 Select Controller-Hub connection mode!

 1. I2C Mode

 2. I3C mode

 Enter your choice :-
 2

 I3C Dynamic Address Assignment will occur!

  Only SMBUS and I3C Target devices are supported!!!

 Assign I3C Dynamic Address to Hub!

 Enter a hexadecimal value(eg. 15, 20 etc.)
 :- 0x30

I3C dynamic address assigned to Hub

 P3H2840 I3C HUB Initialization completed


 *********** Main Menu ***************

 1. Check On-board Temperature Sensor Device (P3T1755)

 2. Check External Devices

 3. Enable/Disable/Check IBI

 4. Software Reset

 Enter your choice :-

Enter #1 to check on board temperature sensor in I3C mode on target port 2

Enter #2 to select port 2.
Enter #2 to select I3C mode.
Assign unique dynamic address to the temperature sensor.
Read/write temperature sensor as per choice.
Enter #5 to exit and return to main menu.
Note: Make sure, Controller-hub network is I3C.

 *********** Main Menu ***************

 1. Check On-board Temperature Sensor Device (P3T1755)

 2. Check External Devices

 3. Enable/Disable/Check IBI

 4. Software Reset

 Enter your choice :- 1

  Two temperature sensors (P3T1755) are present on I3C HUB on two different target ports!

 Please Make sure, default configurations match with input selected below by checking <P3H2X4X_config.h>

 1.  Temperature sensor 1 on port 0 (By default, In SMBUS mode)

 2.  Temperature sensor 2 on port 2 (By default, In I3C mode)

 Enter your choice :- 2

 Please Make sure, default configurations match with input selected below by checking <P3H2X4X_config.h>

 1. SMBus

 2. I3C

 Enter your choice :- 2

 P3T1755 Sensor Initialization completed

 Test Temperature Sensor Configurations!

 1. Read current temperature

 2. Get/Set T-low

 3. Get/Set T-high

 4. Check alert

 5. Exit

 Enter your choice :- 1

  Current Temperature(in degree celsius) = 26.812500

 Test Temperature Sensor Configurations!

Enter #2 to check external read/write in SMBUS mode using P3T1755TP device at target port 1

User needs to select option #2 to select external device.
Enter #3 to read/write any externally connected device.
Enter static address as "4c" for P3T1755 device.
Enter #3 to select smbus read/write
Enter #1 to write the below shown details.
Enter #2 to read the written data.
Enter #3 to exit and return to main menu.

*********** Main Menu ***************

 1. Check On-board Temperature Sensor Device (P3T1755)

 2. Check External Devices

 3. Enable/Disable/Check IBI

 4. Software Reset

 Enter your choice :- 2

  Select any option to test external devices: -

 1. Test external RTC device (PCF2131-ARD) in I2C mode

 2. Test external Temperature sensor(P3T1755) in I3C mode

 3. Read/write any externally connected devices

 Enter your choice :- 3

 Enter Static Target Address :- 0x4c

 1. Read/write in I2C mode

 2. Read/write in I3C mode

 3. Read/write in SMbus mode

 Enter your choice :- 3

 1.Write

 2.Read

 3.Exit

 Enter your choice :- 1

 Enter number of bytes to write(eg. 1, 3)- 1

 Enter register number to write- 0x3

 Enter data to write- 44

 Enter target port number (0-7):- 0

 SMBUS Write complete!

 1.Write

 2.Read

 3.Exit

 Enter your choice :- 2

 Read Data:- 44

 1.Write

 2.Read

 3.Exit

 Enter your choice :-

Enter #4 to reset the HUB device.

User needs to select option #4 to reset the hub device and re-configure the hub- controller network.


 *********** Main Menu ***************

 1. Check On-board Temperature Sensor Device (P3T1755)

 2. Check External Devices

 3. Enable/Disable/Check IBI

 4. Software Reset

 Enter your choice :- 4

  HUB reset completed

 P3H2840 I3C HUB Initialization completed

 Select Controller-Hub connection mode!

 1. I2C Mode

 2. I3C mode

 Enter your choice :-

Re-configure the hub network in I2C mode

After hub reset, Enter #1 to choose the Controller-Hub network as I2C mode.

 Enter your choice :-
 1

  Only I2C and SMBUS Target devices are supported!!!

 I2C static slave address will be used.

 P3H2840 I3C HUB Initialization completed


 *********** Main Menu ***************

 1. Check On-board Temperature Sensor Device (P3T1755)

 2. Check External Devices

 3. Enable/Disable/Check IBI

 4. Software Reset

 Enter your choice :-

Enter #1 for testing on die temperature sensor in Smbus mode on target port 0

Enter #1 to select port 0.
Enter #1 to select smbus mode.
Read/write temperature sensor as per choice.
Enter #5 to exit and return to main menu.

 *********** Main Menu ***************

 1. Check On-board Temperature Sensor Device (P3T1755)

 2. Check External Devices

 3. Enable/Disable/Check IBI

 4. Software Reset

 Enter your choice :- 1

  Two temperature sensors (P3T1755) are present on I3C HUB on two different target ports!

 Please Make sure, default configurations match with input selected below by checking <P3H2X4X_config.h>

 1.  Temperature sensor 1 on port 0 (By default, In SMBUS mode)

 2.  Temperature sensor 2 on port 2 (By default, In I3C mode)

 Enter your choice :- 1

 Please Make sure, default configurations match with input selected below by checking <P3H2X4X_config.h>

 1. SMBus

 2. I3C

 Enter your choice :- 1

 P3T1755 Sensor Initialization completed

 Test Temperature Sensor Configurations!

 1. Read current temperature

 2. Get/Set T-low

 3. Get/Set T-high

 4. Check alert

 5. Exit

 Enter your choice :- 1

  Current Temperature(in degree celsius) = 25.062500

 Test Temperature Sensor Configurations!

 1. Read current temperature

 2. Get/Set T-low

 3. Get/Set T-high

 4. Check alert

 5. Exit

 Enter your choice :- 5
Bye....!

 *********** Main Menu ***************

 1. Check On-board Temperature Sensor Device (P3T1755)

 2. Check External Devices

 3. Enable/Disable/Check IBI

 4. Software Reset

 Enter your choice :-


