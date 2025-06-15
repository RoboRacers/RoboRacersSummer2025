# Description of the project
The goal of this is to create a basic example of a Finite State Machine (FSM) for the 16481 FTC 24-25 Into the Deep robot.

## What is an FSM?
This is more for myself but it's helpful to think of an FSM as a switchboard but for a robot. Essentialy, once you switch on/off one of the switches, some actions will happen. If one of the switches break, the others can still operate. This allows the robot to essentially have failsafes and makes the operation a bit easier.

## Included States
* IDLE
This one is pretty self explanatory but it's the state that the robot is set into whenever the autonomous opmode is run.
"This is the state the robot starts in" - Annoying Anon.
* INTAKE
This one begins the intake mechanism of the robot. It's not currently functional so it simply updates the telemetry
* OUTTAKE
Read above
* MOVE_FORWARD
This one drives the robot forward for as long as the state is active
* TURN
This one turns the robot for as long as the state is active
* STOP
Self-explanatory