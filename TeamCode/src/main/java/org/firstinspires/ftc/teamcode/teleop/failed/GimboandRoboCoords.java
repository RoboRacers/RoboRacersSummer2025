package org.firstinspires.ftc.teamcode.teleop.failed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import android.graphics.Point;

@Disabled
public class GimboandRoboCoords extends OpMode{

    // Camera and object coordinates
    double xCenterCamera;
    double yCenterCamera;
    double xCoordObj;
    double yCoordObj;
    double distanceFromCamera;
    double cameraHeight;

    public double newTargetAngle;

    // Robot offset from camera in inches (set your actual offset here)
    double robotOffsetX = 0;
    double robotOffsetY = 0;

    public Point getCameraCoordinate() {
        double xCoord = 0;
        double yCoord = Math.sqrt(Math.pow(distanceFromCamera, 2) - Math.pow(cameraHeight, 2)); // Pythagorean theorem

        double xError = xCoordObj - xCenterCamera;
        double pixelToInchRatio = yCoord / yCoordObj;
        xCoord = pixelToInchRatio * xError;

        // Adjust by robot offset to get real-world coordinates relative to robot center
        int realX = (int) Math.round(xCoord - robotOffsetX);
        int realY = (int) Math.round(yCoord - robotOffsetY);

        return new Point(realX, realY);
    }

    private Follower follower;
    private DcMotor slidesMotor;
    private FtcDashboard dashboard;

    double motorPower;
    double maxPower;

    // PID constants for slides motor
    public static double kP = 0.08;
    public static double kI = 0.00;
    public static double kD = 0.0001;
    public static double kF = 0.0;

    // Target slides extension in inches (used as "angle" in your original code)
    public double targetAngle = 0.0;
    public double initTarget = 0.0;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private ElapsedTime timer = new ElapsedTime();
    private final Pose startPose = new Pose(0, 0, 0);

    /** Converts inches to encoder ticks for slides motor **/
    public double target(double inches) {
        return (inches * 30.2439); // your conversion factor
    }
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        follower.setStartingPose(startPose);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize dashboard and telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer.reset();
        targetAngle = 15;
    }

    @Override
    public void init_loop() {
        double currentPosition = slidesMotor.getCurrentPosition();
        double error = target(targetAngle) - currentPosition;

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        if (Math.abs(motorPower) > Math.abs(maxPower)) {
            maxPower = motorPower;
        }

        slidesMotor.setPower(motorPower);

        lastError = error;
        lastTarget = target(targetAngle);
        timer.reset();
    }

    @Override
    public void start() {
        initTarget = targetAngle;
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Get robot-relative camera coordinate
        Point cameraCoords = getCameraCoordinate();

        if (gamepad1.cross) {
            follower.setStartingPose(startPose);
            newTargetAngle = cameraCoords.y;
            if (newTargetAngle >= 17) {
                newTargetAngle = 17;
            } else if (newTargetAngle <= 0) {
                newTargetAngle = 0;
            }
        }

        targetAngle = newTargetAngle - follower.getPose().getX();

        // PID loop for slides motor
        double currentPosition = slidesMotor.getCurrentPosition();
        double error = target(targetAngle) - currentPosition;

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        if (Math.abs(motorPower) > Math.abs(maxPower)) {
            maxPower = motorPower;
        }

        slidesMotor.setPower(motorPower);

        lastError = error;
        lastTarget = target(targetAngle);
        timer.reset();

        // Existing follower angle update logic (unchanged)
        targetAngle = initTarget - follower.getPose().getX();
        if (targetAngle >= 17){
            targetAngle = 17;
        }
        else if (targetAngle <= 0){
            targetAngle = 0;
        }
        // Telemetry updates
        telemetry.addData("Camera X", cameraCoords.x);
        telemetry.addData("Camera Y (Slides Target inches)", cameraCoords.y);
        telemetry.addData("Slides Target (ticks)", target(targetAngle));
        telemetry.addData("Slides Current (ticks)", currentPosition);
        telemetry.addData("Slides Motor Power", motorPower);
        telemetry.addData("Max Power Used", maxPower);

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
        dashboard.getTelemetry();
    }

    @Override
    public void stop() {
        // Optionally stop the slides motor
        slidesMotor.setPower(0);
    }
}
