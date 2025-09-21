package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Run 2 Motors To     v ggle", group="Linear Opmode")
public class ConstantMotors extends LinearOpMode {

    // Declare motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        // Initialize motors from configuration
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Set motor directions
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE); // One motor reversed

        // Wait for start button
        waitForStart();

        boolean running = true; // Tracks motor state
        boolean trianglePressed = false;
        boolean squarePressed = false;

        while (opModeIsActive()) {

            // Detect TRIANGLE press to stop motors
            if (gamepad1.triangle && !trianglePressed) {
                running = false;
                trianglePressed = true;
            } else if (!gamepad1.triangle) {
                trianglePressed = false;
            }

            // Detect SQUARE press to resume motors
            if (gamepad1.square && !squarePressed) {
                running = true;
                squarePressed = true;
            } else if (!gamepad1.square) {
                squarePressed = false;
            }

            // Set motor power based on running state
            if (running) {
                leftMotor.setPower(1.0);
                rightMotor.setPower(1.0);
            } else {
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
            }

            telemetry.addData("Motors Running", running);
            telemetry.update();
        }
    }
}
