package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Control with Gamepad", group="Tutorials")
public class TestServo extends LinearOpMode {

    // Declare a Servo object
    private Servo myServo;

    // Define the initial servo position (e.g., 0.5 for the center)
//    private double servoPosition = 0.5;

    // Define constants for the servo's movement increments or decrements
//    private static final double SERVO_INCREMENT = 0.01; // Adjust this value for smoother or faster movement

    @Override
    public void runOpMode() {
        // Initialize the servo from the hardware map
        // Replace "my_servo" with the name you configured for your servo in the Robot Controller app
        myServo = hardwareMap.get(Servo.class, "turret");

        // Set the initial position of the servo
//        myServo.setPosition(servoPosition);

        // Display initial servo position on the Driver Station
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Servo Position", servoPosition);
//        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Control the servo with the D-pad UP button (increase position)
            if(gamepad1.x)
                myServo.setPosition(gamepad1.left_stick_y);
            // Display the current servo position on the Driver Station
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.update();
        }
    }
}
