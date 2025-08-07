package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Deposit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * WARNING:
 * THIS CODE IS PRIMARILY UNCHANGED FROM THE INTAKE CLAW CODE.
 * IT HAS NOT BEEN COMPLETELY ADJUSTED FOR THE DEPOSIT YET.
 *
 * THE CURRENT VALUES ARE ALL SET TO ZERO.
 * DO NOT RUN THIS CODE UNTIL THE TRUE VALUES HAVE BEEN CONFIRMED.
 */

public class Claw {

    private Servo clawServo;
    private final double OPEN_POSITION = 0;
    private final double CLOSED_POSITION = 0;

    Telemetry myTelemetry;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        clawServo = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }

    public void open() {
        clawServo.setPosition(OPEN_POSITION);
        myTelemetry.addData("Deposit Claw Open, Position ", getPosition());
        myTelemetry.update();
    }

    public void close() {
        clawServo.setPosition(CLOSED_POSITION);
        myTelemetry.addData("Deposit Claw Close, Position ", getPosition());
        myTelemetry.update();
    }

    public double getPosition() {
        return clawServo.getPosition();
    }
}