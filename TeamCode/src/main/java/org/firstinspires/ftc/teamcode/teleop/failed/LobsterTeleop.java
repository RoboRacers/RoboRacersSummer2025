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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.teleop.CombinedHSVandAnglePipeline;
import org.firstinspires.ftc.teamcode.teleop.LobsterCup.DepositAutomate;
import org.firstinspires.ftc.teamcode.teleop.LobsterCup.IntakeWithVision;
import org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake.Vision.PixelToDistanceMapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@TeleOp(name = "LobsterTeleop", group = "Combined")
public class LobsterTeleop extends OpMode {

    private Follower follower;

    private FtcDashboard dashboard;


    IntakeWithVision intake = new IntakeWithVision();

    DepositAutomate deposit = new DepositAutomate();


    private ElapsedTime timer = new ElapsedTime();
    private final Pose startPose = new Pose(0, 0, 0);

    public Pose capturedPose = new Pose(0, 0, 0);


    OpenCvCamera camera;
    CombinedHSVandAnglePipeline pipeline;

    public double target(double inches) {
//        return (inches * 27.92);
//return (inches*28.16506); //if slides go all the way to 21.5 inches
        return(inches  * 28.229);// if slides go below 19 inches
        // return (inches * 35.294);
    }
//    private boolean lastX = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastLeftBumper = false;
    public DigitalChannel topLimitSwitch;

    DcMotor intakeSlide;

    enum VisionState {
        IDLE, RETRACTING, SNAPSHOT_PENDING
    }


    private VisionState visionState = VisionState.IDLE;

    private double centerXpos = 0, centerYpos = 0, varForwardDistance = 0;



    double[][] calibrationData = new double[][]{
            {600, 775, 6,  -13, 11},
            {1254,  788, 7, -1.2,  12},
            {1695,  752, 7, 9.6, 20},
            {740, 478, 14, -11.75, 15},
            {905, 509, 12.5, -9, 14},
            {1110, 523, 13, -3, 15},
            {1278, 531, 12.5, 2.2, 20},
            {1486, 522, 14, 10, 25},
            {800, 348, 22, 9.25, 24},
            {1064, 347, 23.5, -1, 24},
            {1307, 394, 22, 8.5, 30}
    };
    PixelToDistanceMapper mapper = new PixelToDistanceMapper(calibrationData);


    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);


        intake.init(hardwareMap);
        deposit.init(hardwareMap);

        topLimitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");


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

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());




        timer.reset();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

        if (gamepad1.b){
            pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.RED);
            telemetry.addLine("RED");
        }
        else if (gamepad1.x){
            pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.BLUE);
            telemetry.addLine("Blue");
        }
        else {
            pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.YELLOW);
            telemetry.addLine("Yellow");

        }

        telemetry.update();






        intake.setSlidesTargetInches(200);
        deposit.setSlidesTargetInches(500);


        intake.update();
        deposit.update();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Handle X press for detection
//        boolean detectPressed = gamepad1.x && !lastX;
//        lastX = gamepad1.x;
//        telemetry.addData("x pressed?", detectPressed);

        boolean detectXPressed = gamepad2.x && !lastX;
        lastX = gamepad2.x;
        telemetry.addData("Gamepad2 X Pressed?", detectXPressed);

        boolean detectYPressed = gamepad2.y && !lastY;
        lastY = gamepad2.y;
        telemetry.addData("Gamepad2 Y Pressed?", detectYPressed);

        boolean detectAPressed = gamepad2.a && !lastA;
        lastA = gamepad2.a;
        telemetry.addData("Gamepad2 A Pressed?", detectAPressed);

        boolean detectBPressed = gamepad2.b && !lastB;
        lastB = gamepad2.b;
        telemetry.addData("Gamepad2 B Pressed?", detectBPressed);

        boolean detectLeftBumperPressed = gamepad2.left_bumper && !lastLeftBumper;
        lastLeftBumper = gamepad2.left_bumper;
        telemetry.addData("Gamepad2 Left Bumper Pressed?", detectLeftBumperPressed);


        if(topLimitSwitch.getState() == false){
            telemetry.addLine("SWITCH CLICKED");
            intake.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        switch (visionState) {
            case IDLE:
                if (lastY) {
                    intake.setSlidesTargetInches(0);
                    visionState = VisionState.RETRACTING;
//                    retractStartTime = timer.seconds();
                }
                break;
            case RETRACTING:
                if (Math.abs(intakeSlide.getCurrentPosition()) < 150) {
                    intakeSlide.setPower(0);
                    visionState = VisionState.SNAPSHOT_PENDING;
                    pipeline.triggerSnapshot();
                }
                break;
            case SNAPSHOT_PENDING:
                if (pipeline.hasProcessedSnapshot()) {
                    if (pipeline.getCenter() != null) {
                        PixelToDistanceMapper.DistanceResult result = mapper.getDistanceFromPixel(
                                pipeline.getCenter().x, pipeline.getCenter().y
                        );
                        centerXpos = pipeline.getCenter().x;
                        centerYpos = pipeline.getCenter().y;
                        varForwardDistance = result.forwardDist;
                        capturedPose = follower.getPose();
                        double headingAtCapture = capturedPose.getHeading();
                        double offsetX = varForwardDistance * Math.cos(headingAtCapture);
                        double offsetY = varForwardDistance * Math.sin(headingAtCapture);
                        double targetX = capturedPose.getX() + offsetX;
                        double targetY = capturedPose.getY() + offsetY;
                        double robotX = follower.getPose().getX();
                        double robotY = follower.getPose().getY();
                        double robotHeading = follower.getPose().getHeading();
                        double dx = targetX - robotX;
                        double dy = targetY - robotY;
                        double forwardComponent = dx * Math.cos(robotHeading) + dy * Math.sin(robotHeading);
                        intake.setSlidesTargetInches(Math.max(0, Math.min(18.5, intake.inchesToTicks(forwardComponent-5)))); //need to consider turrt position

                        intake.setHeightPosition(0.2822);
                        intake.setTurret(0.23);

//                        newValue = (((oldValue - 0.6)/(0)

                        try {
                            sleep(100);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    } else {
                        intake.setSlidesTargetInches(0);
                    }
                    visionState = VisionState.IDLE;
                }
                break;
        }





        intake.updateTransferState(lastX);
//        intake.runVisionLogic(lastY);
        deposit.updateScoreState(lastLeftBumper);
        deposit.updateBasketTransferState(lastB);
        deposit.updateSpecimenTransferState(lastA);





        // Drive Control
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.75,
                true
        );
        follower.update();

        intake.update();
        deposit.update();

        intake.telemetry(telemetry);
        deposit.telemetry(telemetry);


        // Telemetry
telemetry.update();
    }

    @Override
    public void stop() {
        camera.stopStreaming();
    }
}
