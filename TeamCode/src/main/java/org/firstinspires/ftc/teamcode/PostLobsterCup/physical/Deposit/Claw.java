package org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {

    private Servo clawServo;
    private double OPEN_POSITION = 0.0;
    private double CLOSED_POSITION = 0.0;

    private final Telemetry myTelemetry;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        clawServo = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }

    public void open() {
        clawServo.setPosition(OPEN_POSITION);
        myTelemetry.addData("Claw", "OPEN (%.2f)", OPEN_POSITION);
        myTelemetry.update();
    }

    public void close() {
        clawServo.setPosition(CLOSED_POSITION);
        myTelemetry.addData("Claw", "CLOSED (%.2f)", CLOSED_POSITION);
        myTelemetry.update();
    }

    public double getPosition() {
        return clawServo.getPosition();
    }

    // Allow tuning at runtime
    public void setOpenPosition(double position) {
        OPEN_POSITION = position;
    }

    public void setClosedPosition(double position) {
        CLOSED_POSITION = position;
    }
}