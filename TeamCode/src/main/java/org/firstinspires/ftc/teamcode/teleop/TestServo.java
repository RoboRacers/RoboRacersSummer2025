package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "tessttyyyyy Servo")
public class TestServo extends LinearOpMode {

    private Servo servo;
    private double servoPosition = 0.5;  // Start at middle
    private final double INCREMENT = 0.01;  // Step size
    private final double MIN_POS = 0.0;
    private final double MAX_POS = 1.0;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(servoPosition);

        telemetry.addLine("Ready - Press Play");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Right trigger increases position
            if (gamepad1.right_trigger > 0.1) {
                servoPosition += INCREMENT;
            }

            // Left trigger decreases position
            if (gamepad1.left_trigger > 0.1) {
                servoPosition -= INCREMENT;
            }

            // Clamp the position to [0.0, 1.0]
            servoPosition = Math.max(MIN_POS, Math.min(MAX_POS, servoPosition));

            // Set the servo position
            servo.setPosition(servoPosition);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();

            sleep(20); // small delay to avoid flooding
        }
    }
}
