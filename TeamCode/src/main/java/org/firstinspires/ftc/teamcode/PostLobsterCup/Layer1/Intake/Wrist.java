package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist {

    private Servo wristServo;
    private final double ZERO_DEGREE_POSITION = 0.0;
    private final double NINETY_DEGREE_POSITION = 0.5;
    private final double ONE_EIGHTY_DEGREE_POSITION = 1.0;
    private final double TRANSFER_POSITION = 1;

    Telemetry myTelemetry;

    public Wrist(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        wristServo = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }

    public void moveToZeroDegrees() {
        wristServo.setPosition(ZERO_DEGREE_POSITION);
        myTelemetry.addData("Wrist Moved to ", getPosition());
        myTelemetry.update();
    }

    public void moveToNinetyDegrees() {
        wristServo.setPosition(NINETY_DEGREE_POSITION);
        myTelemetry.addData("Wrist Moved to ", getPosition());
        myTelemetry.update();
    }

    public void moveToOneEightyDegrees() {
        wristServo.setPosition(ONE_EIGHTY_DEGREE_POSITION);
        myTelemetry.addData("Wrist Moved to ", getPosition());
        myTelemetry.update();
    }

    public void moveToPosition(double position) {
        // Clamp the input to valid range [0.0, 1.0]
        position = Math.max(0.0, Math.min(1.0, position));
        wristServo.setPosition(position);
        myTelemetry.addData("Wrist", "Moved to custom position: %.2f", position);
        myTelemetry.update();
    }

    public void moveToAngle(double angle) {

        double pos = ( (angle - 0) / (270 - 0) ) * (.8244 - 0) + 0;
        wristServo.setPosition(pos);
        myTelemetry.addData("Wrist", "Moved to angle: %.1f° (position: %.2f)", angle, pos);
        myTelemetry.update();
    }

    public void moveToTransferPosition() {
        wristServo.setPosition(TRANSFER_POSITION);
        myTelemetry.addData("Wrist Moved to ", getPosition());
        myTelemetry.update();
    }

    public double getPosition() {
        double pos = wristServo.getPosition();
        return pos;
    }
}
