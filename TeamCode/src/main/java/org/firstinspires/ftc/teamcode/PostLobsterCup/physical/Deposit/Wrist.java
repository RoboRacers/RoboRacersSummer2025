package org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist {

    private Servo wristServo;

    // Tunable preset positions
    private double ZERO_DEGREE_POSITION = 0.0;
    private double NINETY_DEGREE_POSITION = 0.5;
    private double ONE_EIGHTY_DEGREE_POSITION = 1.0;
    private double TRANSFER_POSITION = 1.0;

    // Servo angle mapping constants
    private double SERVO_MIN_POS = 0.0;
    private double SERVO_MAX_POS = 0.8244; // change if different
    private double SERVO_MIN_ANGLE = 0.0;
    private double SERVO_MAX_ANGLE = 270.0;

    private final Telemetry myTelemetry;

    public Wrist(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        wristServo = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }

    public void moveToZeroDegrees() {
        setPosition(ZERO_DEGREE_POSITION, "ZERO degrees");
    }

    public void moveToNinetyDegrees() {
        setPosition(NINETY_DEGREE_POSITION, "NINETY degrees");
    }

    public void moveToOneEightyDegrees() {
        setPosition(ONE_EIGHTY_DEGREE_POSITION, "ONE EIGHTY degrees");
    }

    public void moveToTransferPosition() {
        setPosition(TRANSFER_POSITION, "TRANSFER position");
    }

    public void moveToPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        wristServo.setPosition(position);
        myTelemetry.addData("Wrist", "Moved to custom position: %.2f", position);
        myTelemetry.update();
    }

    public void moveToAngle(double angle) {
        // Map angle range to servo position range
        double pos = ((angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) *
                (SERVO_MAX_POS - SERVO_MIN_POS) + SERVO_MIN_POS;
        moveToPosition(pos);
        myTelemetry.addData("Wrist", "Moved to angle: %.1f° (position: %.2f)", angle, pos);
        myTelemetry.update();
    }

    public double getPosition() {
        return wristServo.getPosition();
    }

    // Allow tuning preset positions
    public void setZeroDegreePosition(double pos) { ZERO_DEGREE_POSITION = pos; }
    public void setNinetyDegreePosition(double pos) { NINETY_DEGREE_POSITION = pos; }
    public void setOneEightyDegreePosition(double pos) { ONE_EIGHTY_DEGREE_POSITION = pos; }
    public void setTransferPosition(double pos) { TRANSFER_POSITION = pos; }

    // Allow tuning servo mapping
    public void setServoRange(double minPos, double maxPos, double minAngle, double maxAngle) {
        SERVO_MIN_POS = minPos;
        SERVO_MAX_POS = maxPos;
        SERVO_MIN_ANGLE = minAngle;
        SERVO_MAX_ANGLE = maxAngle;
    }

    // Private helper for telemetry consistency
    private void setPosition(double position, String description) {
        wristServo.setPosition(position);
        myTelemetry.addData("Wrist", "Moved to %s (%.2f)", description, position);
        myTelemetry.update();
    }
}
