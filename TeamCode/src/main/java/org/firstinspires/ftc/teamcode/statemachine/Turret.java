package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// The Drive class is a simple abstraction layer to control a two-motor drive system.
public class Turret {
    // These are the motor references for the left and right drive motors.
    private Servo turretServo;
    private double backPosition = 0;
    private double forwardPosition = 1;


    // Constructor: Initializes the left and right motors with the ones passed in.
    public Turret(Servo turretServo) {
        this.turretServo = turretServo;
    }
    public double clamp(double position){
        return Math.max(0.0, Math.min(1.0, position));
    }
    //Turret set to any position
    public void setArmPosition(double position) {
        turretServo.setPosition(clamp(position));
    }
    public void back() {
        turretServo.setPosition(backPosition);
    }
    public void forward() {
        turretServo.setPosition(forwardPosition);
    }
}
