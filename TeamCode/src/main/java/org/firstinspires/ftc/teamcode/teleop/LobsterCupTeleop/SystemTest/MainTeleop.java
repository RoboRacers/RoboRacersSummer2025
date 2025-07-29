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
import org.firstinspires.ftc.teamcode.teleop.PixelToDistanceMapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "A Wrok", group = "A")
public class MainTeleop extends OpMode {

    private Follower follower;

    private FtcDashboard dashboard;

    private double lastTime = 0;


    private double filteredDerivative = 0;
    private static final double MAX_INTEGRAL = 1000; // or some tuned value


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


    private ElapsedTime timer = new ElapsedTime();

    public static double kP = 0.008;
    public static double kI = 0.00007;
    public static double kD = 0.000001;
    public static double kF = 0.0;

    private double maxPower = 0;


    private double maxPower1 = 0;

    private final Pose startPose = new Pose(0, 0, 0);

    public Pose capturedPose = new Pose(0, 0, 0);


    OpenCvCamera camera;
    CombinedHSVandAnglePipeline pipeline;

    private double waitStartTime = 0;

    public double target(double inches) {
//        return (inches * 27.92);
//return (inches*28.16506); //if slides go all the way to 21.5 inches
        return(inches  * 28.229);// if slides go below 19 inches
        // return (inches * 35.294);
    }
//    private boolean lastX = false;
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
//        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");


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


        targetAngle = 200;
        deposit.targetInches = 500;
        // Use a shared timer for both PID loops
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt > 0) {
            // ===== Intake Slide PID =====
            double currentIntakePos = intake.intakeSlide.getCurrentPosition();
            double intakeError = targetAngle - currentIntakePos;

            integralSum += intakeError * dt;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));

            double intakeDerivative = (intakeError - lastError) / dt;
            filteredDerivative = 0.8 * filteredDerivative + 0.2 * intakeDerivative;

            double intakePower = (kP * intakeError) + (kI * integralSum) + (kD * filteredDerivative);
            intakePower = Math.max(-1.0, Math.min(1.0, intakePower));

            intake.intakeSlide.setPower(intakePower);
            lastError = intakeError;

            // ===== Deposit (Vertical Slides) PID =====
            double currentVertSlidePos = deposit.verticalSlides.getCurrentPosition();
            double depositError = deposit.targetInches - currentVertSlidePos;

            deposit.integralSum += depositError * dt;
            deposit.integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, deposit.integralSum));

            double depositDerivative = (depositError - deposit.lastError) / dt;
            deposit.filteredDerivative = 0.8 * deposit.filteredDerivative + 0.2 * depositDerivative;

            double depositPower = (deposit.kP * depositError) +
                    (deposit.kI * deposit.integralSum) +
                    (deposit.kD * deposit.filteredDerivative);
            depositPower = Math.max(-1.0, Math.min(1.0, depositPower));

            deposit.verticalSlides.setPower(depositPower);
            deposit.lastError = depositError;
        }


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

        boolean detectBPressed = gamepad2.right_bumper && !lastRightBumper;
        lastRightBumper = gamepad2.right_bumper;
        telemetry.addData("Gamepad2 B Pressed?", detectBPressed);

        boolean detectLeftBumperPressed = gamepad2.left_bumper && !lastLeftBumper;
        lastLeftBumper = gamepad2.left_bumper;
        telemetry.addData("Gamepad2 Left Bumper Pressed?", detectLeftBumperPressed);


        if(!topLimitSwitch.getState()){
            telemetry.addLine("SWITCH CLICKED");
            intake.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        switch (visionState) {
            case IDLE:
                if (detectYPressed) {
                     targetAngle = 100;
                     intake.turret.setPosition(0.3);
                    visionState = VisionState.RETRACTING;
//                    retractStartTime = timer.seconds();
                }
                break;
            case RETRACTING:
                if (Math.abs(intake.intakeSlide.getCurrentPosition()) < 150) {
                    intake.intakeSlide.setPower(0);
                    intake.heightServo.setPosition(0.45);
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
                        if (waitStartTime == 0) {
                            waitStartTime = timer.seconds();


                            targetAngle = (Math.max(0, Math.min(600, intake.inchesToTicks(forwardComponent)))); //need to consider turrt position
                        }
                        else if (timer.seconds() - waitStartTime > 1.25) {

                            intake.heightServo.setPosition(0.2822);
                            intake.turret.setPosition(0.23);
                            visionState = VisionState.IDLE;
                            waitStartTime = 0;

                        }
                        break;
//                        newValue = (((oldValue - 0.6)/(0)

//                        try {
//                            sleep(100);
//                        } catch (InterruptedException e) {
//                            throw new RuntimeException(e);
//                        }
                    } else {
                        targetAngle = 20;
                        visionState = VisionState.IDLE;
                    }

                }
                telemetry.addLine("Wokring");
                break;
        }



        switch (transferState) {
            case IDLE:
                if (detectXPressed) {
                    // Start transfer sequence
                    intake.clawServo.setPosition(0.40);
                    transferState = TransferState.LIFT;
                }
                break;

            case LIFT:
                if (waitStartTime == 0) {
                    waitStartTime = timer.seconds();
                } else if (timer.seconds() - waitStartTime > 0.5) {
                    // Raise intake to transfer height
                    intake.heightServo.setPosition(0.58); // Up for transfer
                    // Lift claw slightly (optional for clearance)
                    transferState = TransferState.RETRACTING;
                    waitStartTime = 0;
                }
                break;

            case RETRACTING:
                // Pull back or rotate out of submersible
                if (waitStartTime == 0) {
                    waitStartTime = timer.seconds();

                    intake.rotateServo.setPosition(0.57);
                }
                else if (timer.seconds() - waitStartTime > 0.5) {
                    targetAngle = (225); // Target transfer height
                    // Example: move out of transfer angle
                    transferState = TransferState.ROTATE;
                    waitStartTime = 0;
                }
                break;

            case ROTATE:
                // Rotate or drop to final orientation
                intake.turret.setPosition(0.92); // Rotate for transfer
                // Optionally open claw
//                clawServo.setPosition(1.0); // Open to release
                transferState = TransferState.IDLE;
                break;
        }


        switch (score) {
            case IDLE:
                if (detectLeftBumperPressed){
                    score = Score.DROP;
                }
                break;
            case DROP:
                // Your logic here
                if (waitStartTime == 0) {
                    waitStartTime = timer.seconds();
                    deposit.clawServo.setPosition(0.1);
                } else if (timer.seconds() - waitStartTime > 0.5) {
                    score = Score.FLIP;
                    waitStartTime = 0;
                }
                break;
            case FLIP:
                // Your logic here
                if (waitStartTime == 0) {
                    waitStartTime = timer.seconds();
                    deposit.moveWrist(0.1);
                    deposit.moveLift(0.95);
                }
                else if (timer.seconds() - waitStartTime > 0.5) {
                    score = Score.RETRACTING;
                    waitStartTime = 0;
                }
                break;
            case RETRACTING:
                // Your logic here
                
                deposit.targetInches = 100;
                score = Score.IDLE;
                break;
        }

        switch (specimenTransferToBar) {
            case IDLE:
                if (detectAPressed) specimenTransferToBar =SpecimenTransferToBar.CLOSE;

                break;
            case CLOSE:
                if (waitStartTime == 0) {
                    waitStartTime = timer.seconds();
                    deposit.clawServo.setPosition(0.34);
                } else if (timer.seconds() - waitStartTime > 0.5) {
                    intake.clawServo.setPosition(0.6);
                    deposit.moveWrist(0.57);
                    targetAngle = (350);
                    specimenTransferToBar = SpecimenTransferToBar.EXTEND;
                    waitStartTime = 0;
                }

                // The while loop stops everything else, whereas above it allows everything else to run
//                // Close claw to grab specimen
//                deposit.clawServo.setPosition(0.34);
//                timer.reset();
//                while (timer.seconds() < 0.2){
//
//                }
//                deposit.moveWrist(0.57);
//                specimenTransferToBar = SpecimenTransferToBar.EXTEND;
                break;
            case EXTEND:
                if (waitStartTime == 0) {
                    waitStartTime = timer.seconds();
                } else if (timer.seconds() - waitStartTime > 0.5) {

                    // Extend slides to 1347 ticks for specimen height
                    deposit.targetInches = 1100;
                    specimenTransferToBar = SpecimenTransferToBar.FLIP;
                    waitStartTime = 0;
                }
                break;
            case FLIP:

                // Lift to 0.1489 for specimen position
                deposit.moveLift(0.1489);
                // Flip wrist to specimen scoring position
                deposit.moveWrist(0.7);

                specimenTransferToBar = SpecimenTransferToBar.IDLE;
                break;
        }
        switch (basketTransferToBar) {
            case IDLE:
                if (detectBPressed) basketTransferToBar = BasketTransferToBar.CLOSE;
                break;
            case CLOSE:
                if (waitStartTime == 0) {
                    waitStartTime = timer.seconds();
                    deposit.clawServo.setPosition(0.34);
                } else if (timer.seconds() - waitStartTime > 0.5) {
                    intake.clawServo.setPosition(0.6);
                    deposit.moveWrist(0.57);
                    targetAngle = (300);
                    basketTransferToBar = BasketTransferToBar.EXTEND;
                    waitStartTime = 0;
                }
                //could also just do if statements with timer.seconds() instead of another variable

                break;
            case EXTEND:
                // Extend slides to high basket (2742 ticks)
                // Lift to 0.24 for high basket


                deposit.targetInches = 2742;
                basketTransferToBar = BasketTransferToBar.FLIP;
                break;
            case FLIP:
                // Flip wrist to 0.8578 (same as specimen)
                deposit.moveLift(0.24);
                deposit.moveWrist(0.6);
                // Open claw to drop into basket

                basketTransferToBar = BasketTransferToBar.IDLE;
                break;
        }

        // Use a shared timer for both PID loops
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt > 0) {
            // ===== Intake Slide PID =====
            double currentIntakePos = intake.intakeSlide.getCurrentPosition();
            double intakeError = targetAngle - currentIntakePos;

            integralSum += intakeError * dt;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));

            double intakeDerivative = (intakeError - lastError) / dt;
            filteredDerivative = 0.8 * filteredDerivative + 0.2 * intakeDerivative;

            double intakePower = (kP * intakeError) + (kI * integralSum) + (kD * filteredDerivative);
            intakePower = Math.max(-1.0, Math.min(1.0, intakePower));

            intake.intakeSlide.setPower(intakePower);
            lastError = intakeError;

            // ===== Deposit (Vertical Slides) PID =====
            double currentVertSlidePos = deposit.verticalSlides.getCurrentPosition();
            double depositError = deposit.targetInches - currentVertSlidePos;

            deposit.integralSum += depositError * dt;
            deposit.integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, deposit.integralSum));

            double depositDerivative = (depositError - deposit.lastError) / dt;
            deposit.filteredDerivative = 0.8 * deposit.filteredDerivative + 0.2 * depositDerivative;

            double depositPower = (deposit.kP * depositError) +
                    (deposit.kI * deposit.integralSum) +
                    (deposit.kD * deposit.filteredDerivative);
            depositPower = Math.max(-1.0, Math.min(1.0, depositPower));

            deposit.verticalSlides.setPower(depositPower);
            deposit.lastError = depositError;
        }




        if (gamepad1.right_trigger>0.5){
            targetAngle += 10;
        }
        else if (gamepad1.left_trigger>0.5){
            targetAngle -= 10;
        }


        if (gamepad2.right_trigger>0.5){
            intake.clawServo.setPosition(0.40);
        }
        else if (gamepad2.left_trigger >0.5){
            intake.clawServo.setPosition(0.6);
        }



        //later

        if (gamepad1.x){
            intake.turret.setPosition(Math.min(0.95, Math.max(-gamepad1.right_stick_y, 0.1)));
        }



        if (gamepad2.dpad_down){
            intake.heightServo.setPosition(0.2822);
        }

        else if (gamepad2.dpad_up){
            intake.heightServo.setPosition(0.3572);
        }
        else if (gamepad2.left_stick_button){
            intake.heightServo.setPosition(0.58);
        }


        if (gamepad2.dpad_right){
            intake.rotateServo.setPosition(0.8428);
        }

        else if (gamepad2.dpad_left){
            intake.rotateServo.setPosition(0.5583);
        }

        else if (gamepad2.right_stick_button){
            intake.rotateServo.setPosition(gamepad2.left_stick_y);
        }

        // Drive Control
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                (-gamepad1.right_stick_x * 0.75),
                true
        );
        follower.update();
//
//        intake.update();
//        deposit.update();

        telemetry.addData("Vertical Slides Power", deposit.verticalSlides.getPower());
        telemetry.addData("Vertical Slides Pos", deposit.verticalSlides.getCurrentPosition());
        telemetry.addData("Lift Pos L", deposit.liftServoLeft.getPosition());
        telemetry.addData("Lift Pos R", deposit.liftServoRight.getPosition());
        telemetry.addData("Wrist Pos", deposit.wristServo.getPosition());
        telemetry.addData("Claw Pos", deposit.clawServo.getPosition());
        telemetry.addData("Horizontal Slides Power", intake.intakeSlide.getPower());
        telemetry.addData("Turret Pos", intake.intakeSlide.getCurrentPosition());
        telemetry.addData("Arm Pos", intake.heightServo.getPosition());
        telemetry.addData("Rotate Servo Pos", intake.rotateServo.getPosition());
        telemetry.addData("Claw Pos", intake.clawServo.getPosition());
        telemetry.addData("centerx", centerXpos);
        telemetry.addData("centery", centerYpos);
        telemetry.addData("Forward Distance", varForwardDistance);
        telemetry.addData("Slide Target Inches", 5);
        telemetry.addData("Slide Encoder Target", 9);
        telemetry.addData("Current Slide Pos", intake.intakeSlide.getCurrentPosition());
        telemetry.addData("State", visionState);

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
