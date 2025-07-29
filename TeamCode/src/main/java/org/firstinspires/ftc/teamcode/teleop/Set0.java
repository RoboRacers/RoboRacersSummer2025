package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Disabled
@TeleOp(name = "Set Servo 0")
public class Set0 extends LinearOpMode {

    private Servo liftServoLeft;
    private Servo liftServoRight;
    private double servoPosition = 0;  // Start at middle

    @Override
    public void runOpMode() {
        liftServoLeft  = hardwareMap.get(Servo.class, "flipLeft");
        liftServoRight = hardwareMap.get(Servo.class, "flipRight");

        liftServoLeft.setPosition(servoPosition);
        liftServoRight.setPosition(servoPosition);

        telemetry.addLine("Ready - Press Play");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Right trigger increases position
            if (gamepad1.x) {
                servoPosition = 0;
            }
            // Set the servo position
            liftServoLeft.setPosition(servoPosition);
            liftServoRight.setPosition(servoPosition);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();

            sleep(20); // small delay to avoid flooding
        }
    }
}
