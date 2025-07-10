package org.firstinspires.ftc.teamcode.teleop.gimboCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */
//@Disabled
@TeleOp(name = "Gimbo", group = "Examples")
public class Gimbo extends OpMode {
    private Follower follower;
    private DcMotor slidesMotor;
    //    private AnalogInput potentiometer;
    private FtcDashboard dashboard;

    double motorPower;
    double maxPower;

    // Configuration variables (tunable via dashboard)
    public static double kP = 0.08;
    public static double kI = 0.00;
    public static double kD = 0.0001;
    public static double kF = 0.0;
    public static double targetAngle = 0.0; // Target angle in degrees
    public double initTarget = 0.0; // Target angle in degrees
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private ElapsedTime timer = new ElapsedTime();
    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    public double target(double inches){

//        return (inches * 30.2439);

        return (inches * 27.92);
    }


    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        follower.setStartingPose(startPose);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        timer.reset();
        targetAngle = 15;
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {

        double currentAngle = slidesMotor.getCurrentPosition();

        // Calculate error (using angles)
        double error = target(targetAngle) - currentAngle;

        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();

//            double feedForward = kF * Math.sin(Math.toRadians(currentAngle));

        motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        if(Math.abs(motorPower) > Math.abs(maxPower)){
            maxPower = motorPower;
        }

        slidesMotor.setPower(motorPower);

        lastError = error;
        lastTarget = target(targetAngle);
        timer.reset();

        // Telemetry to dashboard

    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        initTarget = targetAngle;
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        targetAngle = initTarget - follower.getPose().getX();


        if (targetAngle >= 17){
            targetAngle = 17;
        }
        else if (targetAngle <= 0){
            targetAngle = 0;
        }


        double currentAngle = slidesMotor.getCurrentPosition();

        // Calculate error (using angles)
        double error = target(targetAngle) - currentAngle;

        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();

//            double feedForward = kF * Math.sin(Math.toRadians(currentAngle));

        motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        if(Math.abs(motorPower) > Math.abs(maxPower)){
            maxPower = motorPower;
        }

        slidesMotor.setPower(motorPower);

        lastError = error;
        lastTarget = target(targetAngle);
        timer.reset();

        // Telemetry to dashboard
        telemetry.addData("Target Angle", target(targetAngle));

        telemetry.addData("Target", targetAngle);
//            telemetry.addData("Current Angle", currentAngle);
        telemetry.addData("Error", error);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.addData("Max Power Used", maxPower);
//            telemetry.addData("Pot Voltage", potentiometer.getVoltage());
//            telemetry.update();
//        telemetry.update();// Important: Update the dashboard

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
        dashboard.getTelemetry();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}