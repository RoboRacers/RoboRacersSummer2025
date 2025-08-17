package org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    private Servo turretServo;

    // 0.92 for transfer
    //0.6 is 0 degrees in the 0 -180 degree conversion for pickup
    // 0 is 150 degrees in the 0-150 degree conversion for pickup
    //0.23 is 90 degrees for pickup
    private final double ZERO_DEGREE_POSITION = 0.6;
    private final double TRANSFER_POSITION = 0.92;
    private final double SUBMERSIBLE_ZONE_POSITION = 0.23;

    Telemetry myTelemetry;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        turretServo = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }

    public void moveToPickPosition() {
        turretServo.setPosition(ZERO_DEGREE_POSITION);
        myTelemetry.addData("Turret", "Moved to PICK position (%.2f)", ZERO_DEGREE_POSITION);
        myTelemetry.update();
    }

    public void moveToTransferPosition() {
        turretServo.setPosition(TRANSFER_POSITION);
        myTelemetry.addData("Turret", "Moved to TRANSFER position (%.2f)", TRANSFER_POSITION);
        myTelemetry.update();
    }

    public void moveToSubmersibleZonePosition() {
        moveToPosition(0.23);
        myTelemetry.addData("Turret", "Moved to SUBMERSIBLE ZONE position (%.2f)", 90);
        myTelemetry.update();
    }

    public void moveToPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        turretServo.setPosition(position);
        myTelemetry.addData("Turret", "Moved to custom position: %.2f", position);
        myTelemetry.update();
    }

    public void moveToInitPos(){
        moveToAngle(0);
        myTelemetry.addData("Turret", "Moved to INIT");
        myTelemetry.update();
    }
    /**
     * Converts an angle in degrees [0,150] to a servo position [0,1]
     * based on the linear mapping defined by 0° -> 0.6 and 90° -> 0.23.
     * Clamps the output between 0 and 1.
     */
    public void moveToAngle(double angle) {
        // Clamp input angle between 0 and 150 degrees
        angle = Math.max(60, Math.min(140, angle));

        // Calculate slope (m) and intercept (b)
        double m = (0.23 - 0.6) / (90 - 0);  // = -0.0041111 approx
        double b = 0.6;

        // Calculate position
        double position = m * angle + b;

        // Clamp servo position between 0 and 1
        position = Math.max(0.01, Math.min(0.99, position));

        moveToPosition(position);
    }

}
