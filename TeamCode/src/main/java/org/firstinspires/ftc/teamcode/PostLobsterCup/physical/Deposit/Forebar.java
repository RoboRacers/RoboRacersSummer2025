package org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Forebar {

    private Servo rightArmServo;
    private Servo leftArmServo;

    // TODO: Tune these positions
    private double TRANSFER_POSITION = 0.0;
    private double DROP_POSITION = 0.0;

    private final Telemetry myTelemetry;

    public Forebar(HardwareMap hardwareMap, Telemetry telemetry, String rightServoName, String leftServoName) {
        rightArmServo = hardwareMap.get(Servo.class, rightServoName);
        leftArmServo = hardwareMap.get(Servo.class, leftServoName);

        // Optional: reverse one servo if they are mirrored
        leftArmServo.setDirection(Servo.Direction.REVERSE);

        myTelemetry = telemetry;
    }

    public void moveToDropPosition() {
        setPosition(DROP_POSITION);
        myTelemetry.addData("AngleArm", "Moved to DROP position (%.2f)", DROP_POSITION);
        myTelemetry.update();
    }

    public void moveToTransferPosition() {
        setPosition(TRANSFER_POSITION);
        myTelemetry.addData("AngleArm", "Moved to TRANSFER position (%.2f)", TRANSFER_POSITION);
        myTelemetry.update();
    }

    public void setPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        rightArmServo.setPosition(position);
        leftArmServo.setPosition(position);
        myTelemetry.addData("AngleArm", "Moved to custom position: %.2f", position);
        myTelemetry.update();
    }

    // Allow updating positions without editing code
    public void setDropPosition(double position) {
        DROP_POSITION = position;
    }

    public void setTransferPosition(double position) {
        TRANSFER_POSITION = position;
    }
}