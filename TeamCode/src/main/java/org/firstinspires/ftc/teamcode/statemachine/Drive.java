package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;

// The Drive class is a simple abstraction layer to control a two-motor drive system.
public class Drive {
    // These are the motor references for the left and right drive motors.
    private DcMotor leftFront;
    private DcMotor rightFront;

    private DcMotor leftBack;
    private DcMotor rightBack;

    // Constructor: Initializes the left and right motors with the ones passed in.
    public Drive(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }

    // Sets both motors to the same power to move the robot forward or backward.
    public void setPower(double power) {
        leftFront.setPower(power);   // Left motor set to given power
        rightFront.setPower(power);  // Right motor set to same power
        leftBack.setPower(power);   // Left motor set to given power
        rightBack.setPower(power);
    }

    // Sets motors to opposite powers to make the robot turn in place.
    // This causes the robot to spin in place (clockwise if power > 0)
    public void turn(double power) {
        leftFront.setPower(power);   // Left motor set to given power
        rightFront.setPower(-power);  // Right motor set to same power
        leftBack.setPower(power);   // Left motor set to given power
        rightBack.setPower(-power);
            }

    // Stops both motors by setting power to 0.
    public void stop() {
        leftFront.setPower(0);   // Left motor set to given power
        rightFront.setPower(0);  // Right motor set to same power
        leftBack.setPower(0);   // Left motor set to given power
        rightBack.setPower(0);
    }
}
