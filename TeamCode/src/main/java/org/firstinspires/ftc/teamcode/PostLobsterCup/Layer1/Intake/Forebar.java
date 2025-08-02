package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Forebar {

    private Servo forebarServo;

    // Tuned servo positions — update as needed
    private final double PICK_POSITION = 0.2822;
    private final double TRANSFER_POSITION = 0.58;
    private final double SUBMERSIBLE_ZONE_POSITION = 0.3572;

    Telemetry myTelemetry;

    public Forebar(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        forebarServo = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }

    public void moveToPickPosition() {
        forebarServo.setPosition(PICK_POSITION);
        myTelemetry.addData("Forebar", "Moved to PICK position (%.2f)", PICK_POSITION);
        myTelemetry.update();
    }

    public void moveToTransferPosition() {
        forebarServo.setPosition(TRANSFER_POSITION);
        myTelemetry.addData("Forebar", "Moved to TRANSFER position (%.2f)", TRANSFER_POSITION);
        myTelemetry.update();
    }

    public void moveToSubmersibleZonePosition() {
        forebarServo.setPosition(SUBMERSIBLE_ZONE_POSITION);
        myTelemetry.addData("Forebar", "Moved to SUBMERSIBLE ZONE position (%.2f)", SUBMERSIBLE_ZONE_POSITION);
        myTelemetry.update();
    }

    public void moveToPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        forebarServo.setPosition(position);
        myTelemetry.addData("Forebar", "Moved to custom position: %.2f", position);
        myTelemetry.update();
    }
}
