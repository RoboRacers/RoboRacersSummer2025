package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * WARNING:
 * THE CURRENT VALUES ARE ALL SET TO ZERO.
 * DO NOT RUN THIS CODE UNTIL THE TRUE VALUES HAVE BEEN CONFIRMED.
 */

public class AngleArm {

    private Servo rightArmServo;
    private Servo leftArmServo;
    private final double TRANSFER_POSITION = 0; // UPDATE WITH PROPER VALUES
    private final double DROP_POSITION = 0; // UPDATE WITH PROPER VALUES

    Telemetry myTelemetry;

    public AngleArm(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        rightArmServo = hardwareMap.get(Servo.class, servoName);
        leftArmServo = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }

    public void moveToDropPosition() {
        rightArmServo.setPosition(DROP_POSITION);
        leftArmServo.setPosition(DROP_POSITION);
        myTelemetry.addData("AngleArm", "Moved to DROP position (%.2f)", DROP_POSITION);
        myTelemetry.update();
    }

    public void moveToTransferPosition() {
        rightArmServo.setPosition(TRANSFER_POSITION);
        leftArmServo.setPosition(TRANSFER_POSITION);
        myTelemetry.addData("AngleArm", "Moved to TRANSFER position (%.2f)", TRANSFER_POSITION);
        myTelemetry.update();
    }

    public void moveToPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        rightArmServo.setPosition(position);
        leftArmServo.setPosition(position);
        myTelemetry.addData("AngleArm", "Moved to custom position: %.2f", position);
        myTelemetry.update();
    }
}
