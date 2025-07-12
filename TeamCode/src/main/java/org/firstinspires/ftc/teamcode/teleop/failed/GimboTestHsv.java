package org.firstinspires.ftc.teamcode.teleop.failed;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.teleop.CombinedHSVandAnglePipeline;
import org.firstinspires.ftc.teamcode.teleop.EnhancedHSVandAngleOpmode;
import org.firstinspires.ftc.teamcode.teleop.PixelToDistanceMapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@TeleOp(name = "URGENT", group = "Apples")

public class GimboTestHsv extends OpMode{

        private Follower follower;
        public Pose currentPose;

        private DcMotor slidesMotor;
        //    private AnalogInput potentiometer;
        public double sampleOffset=0;

        private FtcDashboard dashboard;

        EnhancedHSVandAngleOpmode cam = new EnhancedHSVandAngleOpmode();

        double motorPower;
        double maxPower;

        OpenCvCamera camera;
        CombinedHSVandAnglePipeline pipeline;

        double[][] calibrationData = new double[][]{
                {1140, 401, 13.0,  3.5, 10.0},
                {487,  466, 14.5, -9.8,  7.8},
                {745,  295, 16.2, -5.9, 13.9}
        };

        PixelToDistanceMapper mapper = new PixelToDistanceMapper(calibrationData);
        // Configuration variables (tunable via dashboard)
        public static double kP = 0.011;
        public static double kI = 0.00;
        public static double kD = 0.0000000000000;
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
            return (inches * 27.90697);
        }
        public double targetReverse(double cm){
            return (cm / 27.90697);
        }


        public void camera(){
            pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.RED);
            telemetry.addLine("init before loop");
            for(int i = 0; i<5; i++) {
                pipeline.triggerSnapshot();

                telemetry.addLine("in for loop");

                if (pipeline.hasProcessedSnapshot()) {
                    double angleDeg = pipeline.getTargetAngle();
                    telemetry.addData("Angle (deg)", angleDeg);
                    telemetry.addData("Detected Objects", pipeline.getDetectedObjectsCount());
                    telemetry.addData("pixels", pipeline.getDistance());

                    PixelToDistanceMapper.DistanceResult result = mapper.getDistanceFromPixel(pipeline.getCenter().x, pipeline.getCenter().y);

                    telemetry.addData("Direct Distance", result.directDist);
                    telemetry.addData("Forward", result.forwardDist);
                    telemetry.addData("Horizontal Offset", result.horizOffset);


                    double[] hsv = pipeline.getHSVCenter();
                    if (hsv != null) {
                        initTarget = result.forwardDist;

                        currentPose = follower.getPose();
                        timer.reset();
                        while (timer.seconds() < 1) {
                            sampleOffset = 0;
                        }

                        telemetry.addData("Center HSV", String.format("[%.0f, %.0f, %.0f]", hsv[0], hsv[1], hsv[2]));
                        telemetry.update();
                    } else {
                        telemetry.addData("Bum fix", "");
                        telemetry.update();
                    }
                    try {
                        sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }



        @Override
        public void init() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            pipeline = new CombinedHSVandAnglePipeline();
            camera.setPipeline(pipeline);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(1920, 1200, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Camera Error", errorCode);
                    telemetry.update();
                }
            });

            follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
            follower.setStartingPose(startPose);
            slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            currentPose = follower.getPose();
            // Initialize FTC Dashboard
            dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
            timer.reset();

//        targetAngle = 15;
        }

        /** This method is called continuously after Init while waiting to be started. **/
        @Override
        public void init_loop() {
            // Telemetry to dashboard


        }

        /** This method is called once at the start of the OpMode. **/
        @Override
        public void start() {
//        initTarget = targetAngle;
            follower.startTeleopDrive();
        }

        /** This is the main loop of the opmode and runs continuously after play **/
        @Override
        public void loop() {
            if (gamepad1.cross){
                camera();
            }


            if(gamepad1.dpad_up){
                sampleOffset +=1;
            }
            else if (gamepad1.dpad_down){
                sampleOffset -= 1;
            }

//        targetAngle = initTarget - follower.getPose().getX();
            targetAngle = initTarget - follower.getPose().getX() + currentPose.getX() - sampleOffset;


            if (targetAngle >= 20.5){
                targetAngle = 20.5;
            }
            else if (targetAngle <= 0.5){
                targetAngle = 0.5;
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
//        telemetry.addData("Target Angle", target(targetAngle));
//
//        telemetry.addData("Target", targetAngle);
////            telemetry.addData("Current Angle", currentAngle);
//        telemetry.addData("Error", error);
//        telemetry.addData("Motor Power", motorPower);
//        telemetry.addData("kP", kP);
//        telemetry.addData("kI", kI);
//        telemetry.addData("kD", kD);
//        telemetry.addData("kF", kF);
//        telemetry.addData("Max Power Used", maxPower);
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

            telemetry.addData("Error Difference between camera and slide target", initTarget - targetAngle);



            telemetry.addData("Target Angle", target(targetAngle));
            telemetry.addData("Error PID ticks", error);


            telemetry.addData("Target", targetAngle);
//            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error PID inch", targetReverse(error));
            /* Update Telemetry to the Driver Hub */
            telemetry.update();
            dashboard.getTelemetry();

        }

        /** We do not use this because everything automatically should disable **/
        @Override
        public void stop() {
        }
    }

