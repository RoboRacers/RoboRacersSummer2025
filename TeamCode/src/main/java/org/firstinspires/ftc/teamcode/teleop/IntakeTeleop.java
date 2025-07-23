package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Intake Teleop", group="TeleOp")
public class IntakeTeleop extends OpMode {

    private Servo heightServo;   // Carbon fiber tube height
    private Servo rotateServo;   // Claw rotation
    private Servo clawServo;     // Claw open/close

    private double heightPos = 0.5;   // Start in the middle
    private double rotatePos = 0.5;   // Start in the middle

    @Override
    public void init() {
        heightServo = hardwareMap.get(Servo.class, "heightServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        clawServo   = hardwareMap.get(Servo.class, "clawServo");

        heightServo.setPosition(heightPos);
        rotateServo.setPosition(rotatePos);
        clawServo.setPosition(0.3); // Start closed
    }

    @Override
    public void loop() {
        // Height adjustment with left stick Y
        double joystickY = -gamepad1.left_stick_y;
        heightPos += joystickY * 0.01;
        heightServo.setPosition(heightPos);

        // Claw rotation with bumpers
        if (gamepad1.left_bumper) {
            rotatePos -= 0.01;
        } else if (gamepad1.right_bumper) {
            rotatePos += 0.01;
        }
        rotateServo.setPosition(rotatePos);

        // Claw open/close
        if (gamepad1.a) {
            clawServo.setPosition(1.0); // Open
        } else if (gamepad1.b) {
            clawServo.setPosition(0.3); // Close
        }

        // Telemetry
        telemetry.addData("Height Pos", heightPos);
        telemetry.addData("Rotate Pos", rotatePos);
        telemetry.addData("Claw Pos", clawServo.getPosition());
        telemetry.update();
    }
}
