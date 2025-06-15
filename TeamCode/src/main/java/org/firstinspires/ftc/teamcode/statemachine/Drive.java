package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;

// The Drive class is a simple abstraction layer to control a two-motor drive system.
public class Drive {
    // These are the motor references for the left and right drive motors.
    private DcMotor left;
    private DcMotor right;

    // Constructor: Initializes the left and right motors with the ones passed in.
    public Drive(DcMotor left, DcMotor right) {
        this.left = left;
        this.right = right;
    }

    // Sets both motors to the same power to move the robot forward or backward.
    public void setPower(double power) {
        left.setPower(power);   // Left motor set to given power
        right.setPower(power);  // Right motor set to same power
    }

    // Sets motors to opposite powers to make the robot turn in place.
    // This causes the robot to spin in place (clockwise if power > 0)
    public void turn(double power) {
        left.setPower(-power);  // Left motor moves in reverse
        right.setPower(power);  // Right motor moves forward
            }

    // Stops both motors by setting power to 0.
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }
}
