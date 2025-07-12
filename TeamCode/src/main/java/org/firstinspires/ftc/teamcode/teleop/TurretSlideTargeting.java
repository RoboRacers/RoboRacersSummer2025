package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TurretSlideTargeting", group = "Control")
public class TurretSlideTargeting extends LinearOpMode {

    // Hardware
    private DcMotor slideMotor;
    private Servo turretServo;

    // Constants
    static final double ARM_LENGTH = 6.0; // Tube + claw length in inches
    static final double MAX_SLIDE_EXTENSION = 30.0;
    static final double MIN_YAW_DEG = -180.0;
    static final double MAX_YAW_DEG = 180.0;

    // Slide conversion (inches to motor ticks)
    static final double TICKS_PER_INCH = 100.0; // example value - adjust for your hardware

    @Override
    public void runOpMode() {
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {

            // Example robot pose on field (in inches)
            double robotX = 30;
            double robotY = 30;
            double robotHeading = 90; // degrees

            // Example camera target (relative to robot)
            double camX = -5;  // inches right/left
            double camY = 10;  // inches forward/back

            // Step 1: Convert camera target to field coords
            double[] targetField = toFieldCoords(robotX, robotY, robotHeading, camX, camY);

            // Step 2: Get turret/slide outputs
            ArmControlOutput output = aimToFieldTarget(targetField[0], targetField[1], robotX, robotY, robotHeading);

            // Set slide motor
            int slideTicks = (int)(output.slideExtension * TICKS_PER_INCH);
            slideMotor.setTargetPosition(slideTicks);
            slideMotor.setPower(0.8);

            // Set turret servo
            turretServo.setPosition(output.turretServoPos);

            // Telemetry
            telemetry.addData("Target Field X", targetField[0]);
            telemetry.addData("Target Field Y", targetField[1]);
            telemetry.addData("Slide Ext (in)", output.slideExtension);
            telemetry.addData("Turret Servo Pos", output.turretServoPos);
            telemetry.addData("Slide Target Ticks", slideTicks);
            telemetry.update();
        }
    }

    public double[] toFieldCoords(double robotX, double robotY, double headingDeg, double localX, double localY) {
        double headingRad = Math.toRadians(headingDeg);
        double fieldX = robotX + localX * Math.cos(headingRad) - localY * Math.sin(headingRad);
        double fieldY = robotY + localX * Math.sin(headingRad) + localY * Math.cos(headingRad);
        return new double[]{fieldX, fieldY};
    }

    public static class ArmControlOutput {
        public double slideExtension;
        public double turretServoPos;
    }

    public ArmControlOutput aimToFieldTarget(double targetX, double targetY, double turretX, double turretY, double turretHeadingDeg) {
        ArmControlOutput out = new ArmControlOutput();

        // Offset from turret base to target
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        // Convert to turret-relative frame
        double headingRad = Math.toRadians(turretHeadingDeg);
        double relX = dx * Math.cos(-headingRad) - dy * Math.sin(-headingRad);
        double relY = dx * Math.sin(-headingRad) + dy * Math.cos(-headingRad);

        // Distance to target from turret
        double dist = Math.sqrt(relX * relX + relY * relY);

        // Slide extension
        double slide = dist - ARM_LENGTH;
        slide = Math.max(0, Math.min(slide, MAX_SLIDE_EXTENSION));
        out.slideExtension = slide;

        // Yaw angle
        double yawDeg = Math.toDegrees(Math.atan2(relX, relY));
        yawDeg = Math.max(MIN_YAW_DEG, Math.min(MAX_YAW_DEG, yawDeg));
        out.turretServoPos = (yawDeg - MIN_YAW_DEG) / (MAX_YAW_DEG - MIN_YAW_DEG);

        return out;
    }
}
