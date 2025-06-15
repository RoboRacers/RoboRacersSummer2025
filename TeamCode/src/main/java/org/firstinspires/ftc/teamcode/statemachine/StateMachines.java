package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StateMachines {
    // Holds the current FSM state (like IDLE, MOVE_FORWARD, etc.)
    private AutoState currentState;

    // Timer used by states to manage time-based transitions 
        // (e.g. “after 2 seconds, switch state”)
    public ElapsedTime runtime = new ElapsedTime();

    // Reference to a robot drive class 
        // contain methods like setPower(), turn(), stop())
        // Shared among states for easy control of motors
    public Drive drive;

    // You inject the robot's Drive system into the FSM 
        // when creating the StateMachines object
    public StateMachines(Drive drive) {
        this.drive = drive;
    }

        // Updates the currentState
        // Immediately calls the new state's onEnter() to perform any     
        // initialization for that state You pass in this (the state 
        // machine) and the opMode so the state has full access
    public void setState(AutoState newState, LinearOpMode opMode) {
        currentState = newState;
        currentState.onEnter(this, opMode);
    }

        // Called repeatedly in the main while(opModeIsActive()) loop 
        // Delegates update logic to the current state
    public void update(LinearOpMode opMode) {
        if (currentState != null) {
            currentState.update(this, opMode);
        }
    }
}
