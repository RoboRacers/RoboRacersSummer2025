package org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop.SystemTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.LobsterCup.Deposit;
import org.firstinspires.ftc.teamcode.teleop.LobsterCup.Intake;

@TeleOp(name="All Servo Test", group="Testing")
public class AllServoTest extends LinearOpMode {

    // Declare a Servo object

    Deposit deposit = new Deposit();

    Intake intake = new Intake();

    // Define the initial servo position (e.g., 0.5 for the center)
//    private double servoPosition = 0.5;

    // Define constants for the servo's movement increments or decrements
//    private static final double SERVO_INCREMENT = 0.01; // Adjust this value for smoother or faster movement

    @Override
    public void runOpMode() {
        // Initialize the servo from the hardware map
        // Replace "my_servo" with the name you configured for your servo in the Robot Controller app
        deposit.init(hardwareMap);
        intake.init(hardwareMap);


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

            if(gamepad1.a){
                deposit.moveLift(gamepad1.left_stick_y);
            }
            if (gamepad1.x){
                deposit.moveWrist(gamepad1.left_stick_y);
            }
            if (gamepad1.y){
                deposit.setClawOpen(gamepad1.right_bumper);
            }
            if (gamepad1.b){
                deposit.setSlidesTargetInches(gamepad1.left_stick_y * 2700);
            }

            if (gamepad1.dpad_down){
                intake.setWristIntake(gamepad1.left_stick_y);
            }
            if(gamepad1.dpad_up){
                intake.setHeightPosition(gamepad1.left_stick_y);
            }
            if (gamepad1.dpad_left){
                intake.setRotatePosition(gamepad1.left_stick_y);
            }
            if (gamepad1.dpad_right){
                intake.setClawOpen(gamepad1.left_bumper);
            }
            if (gamepad1.right_trigger > 0.8){
                intake.setSlidesTargetInches(0);
            }

            // Display the current servo position on the Driver Station
          deposit.update();
            deposit.telemetry(telemetry);
            intake.update();


            telemetry.update();
        }
    }
}
