package org.firstinspires.ftc.teamcode.PostLobsterCup.logical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Drive.DriveMotors;
import org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Drive.IMU;

public class Drive {

    private final DriveMotors driveMotors;
    private final IMU imu;

    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        // Link hardware to DriveMotors
        driveMotors = new DriveMotors(hardwareMap, telemetry,
                "frontRight", "frontLeft", "backRight", "backLeft");

        imu = new IMU(); // assuming your IMU class takes these
    }

    /** Example: drive forward at a given power */
    public void driveForward(double power) {
        driveMotors.mecanumDrive(power, 0, 0);
    }

    /** Example: strafe right at a given power */
    public void strafeRight(double power) {
        driveMotors.mecanumDrive(0, power, 0);
    }

    /** Example: turn at a given power */
    public void turn(double power) {
        driveMotors.mecanumDrive(0, 0, power);
    }

    /** Example: stop */
    public void stop() {
        driveMotors.stop();
    }

    /** Encoder reset utility */
    public void resetEncoders() {
        driveMotors.resetEncoders();
    }

    /** Could be called every loop to update telemetry or heading correction */
    public void update() {
        driveMotors.reportEncoders();
        // could also check IMU heading here if needed
    }

    /** Placeholder for an autonomous movement */
    public void driveSomewhere() {
        // This could be replaced with a sequence of drive commands
        driveForward(0.5);
        // add timing or encoder logic here
    }
}