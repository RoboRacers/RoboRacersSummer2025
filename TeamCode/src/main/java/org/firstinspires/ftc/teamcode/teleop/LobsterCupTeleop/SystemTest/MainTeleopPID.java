package org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop.SystemTest;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake.Vision.PixelToDistanceMapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "pidsuperawesomesauce", group = "A")
public class MainTeleopPID extends OpMode {

    private Follower follower;

    private FtcDashboard dashboard;

    private double lastTime = 0;


    public double filteredDerivative = 0;
    public static final double MAX_INTEGRAL = 1000; // or some tuned value


    enum TransferState {
        IDLE, LIFT, RETRACTING, ROTATE
    }

    enum Score {
        IDLE, DROP, FLIP, RETRACTING
    }

    enum SpecimenTransferToBar {
        IDLE, CLOSE, EXTEND, FLIP
    }


    private Score score =   Score.IDLE;
    private SpecimenTransferToBar specimenTransferToBar = SpecimenTransferToBar.IDLE;

    enum BasketTransferToBar {
        IDLE, CLOSE, EXTEND, FLIP
    }
    private BasketTransferToBar basketTransferToBar = BasketTransferToBar.IDLE;


    private TransferState transferState = TransferState.IDLE;


    IntakeWithVision intake = new IntakeWithVision();

    DepositAutomate deposit = new DepositAutomate();
    public double dt;


    public ElapsedTime timer = new ElapsedTime();

    public static double kP = 0.008;
    public static double kI = 0.00007;
    public static double kD = 0.000001;
    public static double kF = 0.0;

    private final Pose startPose = new Pose(0, 0, 0);

    public Pose capturedPose = new Pose(0, 0, 0);


    OpenCvCamera camera;
    CombinedHSVandAnglePipeline pipeline;

    // REPLACED: Use dedicated timers instead of one generic waitStartTime
    private final ElapsedTime visionTimer = new ElapsedTime();
    private final ElapsedTime transferTimer = new ElapsedTime();
    private final ElapsedTime scoreTimer = new ElapsedTime();
    private final ElapsedTime specimenTransferTimer = new ElapsedTime();
    private final ElapsedTime basketTransferTimer = new ElapsedTime();


    public double target(double inches) {
        return(inches  * 28.229);
    }
    private boolean lastX = false;

    public double targetAngle = 0;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    public DigitalChannel topLimitSwitch;

//    DcMotor intakeSlide;

    enum VisionState {
        IDLE, RETRACTING, SNAPSHOT_PENDING
    }


    private VisionState visionState = VisionState.IDLE;
    private double integralSum = 0;


    private double centerXpos = 0, centerYpos = 0, varForwardDistance = 0;
    private double lastError = 0;



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

    @Override
    public void init_loop() {
        pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.YELLOW);

        targetAngle = 200;
        deposit.targetInches = 300;

        double currentTime = timer.seconds();
        dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt > 0) {
            intake.update();
            deposit.update();
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Detect button edge-presses
        boolean detectXPressed = gamepad2.x && !lastX;
        boolean detectYPressed = gamepad2.y && !lastY;
        boolean detectAPressed = gamepad2.a && !lastA;
        boolean detectBPressed = gamepad2.right_bumper && !lastRightBumper;
        boolean detectLeftBumperPressed = gamepad2.left_bumper && !lastLeftBumper;

        lastX = gamepad2.x;
        lastY = gamepad2.y;
        lastA = gamepad2.a;
        lastRightBumper = gamepad2.right_bumper;
        lastLeftBumper = gamepad2.left_bumper;

        if(!topLimitSwitch.getState()){
            intake.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Vision state machine
        switch (visionState) {
            case IDLE:
                if (detectYPressed) {
                    targetAngle = 100;
                    intake.turret.setPosition(0.3);
                    visionState = VisionState.RETRACTING;
                    visionTimer.reset();
                }
                break;
            case RETRACTING:
                if (Math.abs(intake.intakeSlide.getCurrentPosition()) < 150) {
                    intake.intakeSlide.setPower(0);
                    intake.heightServo.setPosition(0.45);
                    visionState = VisionState.SNAPSHOT_PENDING;
                    pipeline.triggerSnapshot();
                    visionTimer.reset();
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
                        if (visionTimer.seconds() == 0) visionTimer.reset();

                        if (visionTimer.seconds() < 1.25) {
                            targetAngle = (Math.max(0, Math.min(600, intake.inchesToTicks(forwardComponent))));
                        } else {
                            intake.heightServo.setPosition(0.2822);
                            intake.turret.setPosition(0.23);
                            visionState = VisionState.IDLE;
                            visionTimer.reset();
                        }
                    } else {
                        targetAngle = 20;
                        visionState = VisionState.IDLE;
                        visionTimer.reset();
                    }
                }
                break;
        }

        // Transfer state machine
        switch (transferState) {
            case IDLE:
                if (detectXPressed) {
                    intake.clawServo.setPosition(0.40);
                    transferState = TransferState.LIFT;
                    transferTimer.reset();
                }
                break;

            case LIFT:
                if (transferTimer.seconds() > 0.5) {
                    intake.heightServo.setPosition(0.58);
                    transferState = TransferState.RETRACTING;
                    transferTimer.reset();
                }
                break;

            case RETRACTING:
                if (transferTimer.seconds() == 0) {
                    intake.rotateServo.setPosition(0.57);
                }
                if (transferTimer.seconds() > 0.5) {
                    targetAngle = 225;
                    transferState = TransferState.ROTATE;
                    transferTimer.reset();
                }
                break;

            case ROTATE:
                intake.turret.setPosition(0.92);
                transferState = TransferState.IDLE;
                break;
        }

        // Score state machine
        switch (score) {
            case IDLE:
                if (detectLeftBumperPressed){
                    score = Score.DROP;
                    scoreTimer.reset();
                }
                break;
            case DROP:
                if (scoreTimer.seconds() == 0) {
                    deposit.clawServo.setPosition(0.1);
                }
                if (scoreTimer.seconds() > 0.5) {
                    score = Score.FLIP;
                    scoreTimer.reset();
                }
                break;
            case FLIP:
                if (scoreTimer.seconds() == 0) {
                    deposit.moveWrist(0.1);
                    deposit.moveLift(0.95);
                }
                if (scoreTimer.seconds() > 0.5) {
                    score = Score.RETRACTING;
                    scoreTimer.reset();
                }
                break;
            case RETRACTING:
                deposit.targetInches = 100;
                score = Score.IDLE;
                break;
        }

        // Specimen Transfer to Bar state machine
        switch (specimenTransferToBar) {
            case IDLE:
                if (detectAPressed) {
                    specimenTransferToBar = SpecimenTransferToBar.CLOSE;
                    specimenTransferTimer.reset();
                }
                break;
            case CLOSE:
                if (specimenTransferTimer.seconds() == 0) {
                    deposit.clawServo.setPosition(0.34);
                }
                if (specimenTransferTimer.seconds() > 0.5) {
                    intake.clawServo.setPosition(0.6);
                    deposit.moveWrist(0.57);
                    targetAngle = 350;
                    specimenTransferToBar = SpecimenTransferToBar.EXTEND;
                    specimenTransferTimer.reset();
                }
                break;
            case EXTEND:
                if (specimenTransferTimer.seconds() > 0.5) {
                    deposit.targetInches = 1100;
                    specimenTransferToBar = SpecimenTransferToBar.FLIP;
                    specimenTransferTimer.reset();
                }
                break;
            case FLIP:
                deposit.moveLift(0.1489);
                deposit.moveWrist(0.7);
                specimenTransferToBar = SpecimenTransferToBar.IDLE;
                break;
        }

        // Basket Transfer to Bar state machine
        switch (basketTransferToBar) {
            case IDLE:
                if (detectBPressed) {
                    basketTransferToBar = BasketTransferToBar.CLOSE;
                    basketTransferTimer.reset();
                }
                break;
            case CLOSE:
                if (basketTransferTimer.seconds() == 0) {
                    deposit.clawServo.setPosition(0.34);
                }
                if (basketTransferTimer.seconds() > 0.5) {
                    intake.clawServo.setPosition(0.6);
                    deposit.moveWrist(0.57);
                    targetAngle = 300;
                    basketTransferToBar = BasketTransferToBar.EXTEND;
                    basketTransferTimer.reset();
                }
                break;
            case EXTEND:
                deposit.targetInches = 2742;
                basketTransferToBar = BasketTransferToBar.FLIP;
                basketTransferTimer.reset();
                break;
            case FLIP:
                deposit.moveLift(0.24);
                deposit.moveWrist(0.6);
                basketTransferToBar = BasketTransferToBar.IDLE;
                break;
        }

        // Shared PID timer logic
        double currentTime = timer.seconds();
        dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt > 0) {
            intake.update();
            deposit.update();
        }

        // Manual turret control via gamepad1 triggers
        if (gamepad1.right_trigger > 0.5){
            targetAngle += 10;
        }
        else if (gamepad1.left_trigger > 0.5){
            targetAngle -= 10;
        }

        // Manual claw control via gamepad2 triggers
        if (gamepad2.right_trigger > 0.5){
            intake.clawServo.setPosition(0.40);
        }
        else if (gamepad2.left_trigger > 0.5){
            intake.clawServo.setPosition(0.6);
        }

        // Turret manual override
        if (gamepad1.x){
            intake.turret.setPosition(Math.min(0.95, Math.max(-gamepad1.right_stick_y, 0.1)));
        }

        // Height servo manual controls
        if (gamepad2.dpad_down){
            intake.heightServo.setPosition(0.2822);
        }
        else if (gamepad2.dpad_up){
            intake.heightServo.setPosition(0.3572);
        }
        else if (gamepad2.left_stick_button){
            intake.heightServo.setPosition(0.58);
        }

        // Rotate servo manual controls
        if (gamepad2.dpad_right){
            intake.rotateServo.setPosition(0.8428);
        }
        else if (gamepad2.dpad_left){
            intake.rotateServo.setPosition(0.5583);
        }
        else if (gamepad2.right_stick_button){
            intake.rotateServo.setPosition(gamepad2.left_stick_y);
        }

        // Drive control
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                (-gamepad1.right_stick_x * 0.75),
                true
        );
        follower.update();
    }

    @Override
    public void stop() {
        camera.stopStreaming();
    }
}
