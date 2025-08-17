package org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {

    private Servo clawServo;
    private final double OPEN_POSITION = 0.6;
    private final double CLOSED_POSITION = 0.4;

    Telemetry myTelemetry;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        clawServo = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }

    public void open() {
        clawServo.setPosition(OPEN_POSITION);
        myTelemetry.addData("Intake Claw Open, Position ", getPosition());
        myTelemetry.update();
    }

    public void close() {
        clawServo.setPosition(CLOSED_POSITION);
        myTelemetry.addData("Intake Claw Close, Position ", getPosition());
        myTelemetry.update();
    }

    public double getPosition() {
        return clawServo.getPosition();
    }
}