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
            // 0.24 for scoring high basket and low basket
             //0.1489 for specimen
            //0.95 for transfer

            if (gamepad1.x){
                deposit.moveWrist(gamepad1.left_stick_y);
            }
            //0.8578 for speciemn high basket and low basket
             //0.1 for transfer
            //0.57 for get out of transfer

            if (gamepad1.y){
                deposit.clawServo.setPosition(gamepad1.left_stick_y);
            }
            // 0.1 is open
             // 0.34 is close

            if (gamepad1.b){
                deposit.setSlidesTargetInches(gamepad1.left_stick_y * 2750);
            }
            // 1347 for specimen
             // 2742 looks like max for high basket
            //1666 low basket
            // 120 for transfer



            if (gamepad1.dpad_down){
                intake.setTurret(gamepad1.left_stick_y);
            }
            // 0.92 for transfer
             //0.6 is 0 degrees in the 0 -180 degree conversion for pickup
            // 0 is 150 degrees in the 0-150 degree conversion for pickup
            //0.23 is 90 degrees for pickup


            if(gamepad1.dpad_up){
                intake.setHeightPosition(gamepad1.left_stick_y);
            }
            // Pickup is 0.2822
             //  Go into submersible 0.3572
             // Up for transfer  is 0.58

            if (gamepad1.dpad_left){
                intake.setRotatePosition(gamepad1.left_stick_y);
            }
            // 0 is pickup for blocsk like this --- closes to pick up
             //

            if (gamepad1.dpad_right){
                intake.clawServo.setPosition(gamepad1.left_stick_y);
            }
            // 0.6 is open
             // 0.41 is closed

            if (gamepad1.right_trigger > 0.8){
                intake.setSlidesTargetInches(gamepad1.left_stick_y*-520);
            }
            // -260 for transfer
             //


            // Display the current servo position on the Driver Station
            deposit.update();
            deposit.telemetry(telemetry);
            intake.update();
            intake.telemetry(telemetry);

            telemetry.update();
        }
    }
}
