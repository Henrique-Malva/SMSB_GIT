# SMSB_GIT
 

This project intends to emulate swarm concepts without the use of any AI.
The goal is to have a main boat (master) that is controlled by the user as any other RC boat.
Meanwhile, the slave boat follows the master, keeping a certain defined distance. The user can then select between 3 following modes, behind, to the side or back and forth (not working).
As it is, the 2 first modes work properly in calm waters in the absence of wind. The addition of IMUs in the boat construction would be the next step in the project to allow for the implementation of closed-loop control of the slave boat's position and movement relative to the master.


BOAT Model: https://www.thingiverse.com/thing:3807800
Propeller Model: https://www.thingiverse.com/thing:1778954
Remote Controller based of: https://www.thingiverse.com/thing:5019581

Materials used (* marks needed upgrades):
Controller:
1x NRF24L01 radio module
1x TFT screen
1x button
2x switches
1x potentiometer
1x joystick
1x 9V battery holder
1x arduino nano

Master Boat:
1x arduino UNO
1x DC motor
1x l298n h-bridge*
1x HC-05 bt module*
1x NRF24L01 radio module
1x temperature and humidity sensor DHT11
2x 3.7V lipo cells*
1x servo

Slave Boat:
1x arduino UNO
1x DC motor
1x Arduino Motor Shield Rev3 (no need for all this)
1x HC-05 bt module*
2x 3.7V lipo cells*
2x servos
1x HC-SR04 sonar*

2x motor axles, 2x guiding tubes for the axles and keeping the water out

Upgrades:
addition of IMUs
better and more controlled energy source (there were brown out problems)
more accurate range finder
h-bridges with higher max current
Bluetooth modules that work properly

