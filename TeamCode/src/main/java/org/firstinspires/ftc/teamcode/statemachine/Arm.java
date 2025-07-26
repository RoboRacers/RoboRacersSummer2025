package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// The Drive class is a simple abstraction layer to control a two-motor drive system.
public class Arm {
    // These are the motor references for the left and right drive motors.
    private Servo armServo;
    private double flipPosition = 0;
    private double unflipPosition = 1;


    // Constructor: Initializes the left and right motors with the ones passed in.
    public Arm(Servo armServo) {
        this.armServo = armServo;
    }
    public double clamp(double position){
        return Math.max(0.0, Math.min(1.0, position));
    }
    public void setArmPosition(double position) {
        armServo.setPosition(clamp(position));
    }
    public void flip() {
        armServo.setPosition(flipPosition);
    }
    public void unflip() {
        armServo.setPosition(unflipPosition);
    }
}
