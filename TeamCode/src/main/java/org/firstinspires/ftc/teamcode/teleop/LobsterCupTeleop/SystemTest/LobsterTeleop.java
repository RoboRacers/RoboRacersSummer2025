package org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop.SystemTest;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name = "GimboDriveSlides", group = "Combined")
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


    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

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


        if (gamepad1.b){
            pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.RED);
        }
        else if (gamepad1.x){
            pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.BLUE);
        }
        else {
            pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.YELLOW);

        }

        timer.reset();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        intake.setSlidesTargetInches(200);
        deposit.setSlidesTargetInches(100);


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

        intake.updateTransferState(lastX);
        intake.runVisionLogic(lastY);
        deposit.updateScoreState(lastLeftBumper);
        deposit.updateBasketTransferState(lastB);
        deposit.updateSpecimenTransferState(lastA);





        // Drive Control
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
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
