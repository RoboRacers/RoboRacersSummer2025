package org.firstinspires.ftc.teamcode.statemachine.statesave;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.statemachine.statesave.SQLiteStateStore;
import org.firstinspires.ftc.teamcode.statemachine.statesave.SQLiteStateStore.RobotStateKey;
import org.firstinspires.ftc.teamcode.statemachine.statesave.StateSubscriber;

@TeleOp(name = "Pose & Slide Tracker sigma", group = "Testing")
public class PoseTrackingOpMode extends LinearOpMode {
    private Follower follower;
    private SQLiteStateStore stateStore;
    private DcMotor slidesMotor;

    private Pose lastPose = new Pose(0, 0, 0);
    private static final double POSITION_THRESHOLD = 0.05;

    private int targetSlideTicks = 0;
    private static final int SLIDE_MIN = 50;
    private static final int SLIDE_MAX = 700;

    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        stateStore = new SQLiteStateStore(hardwareMap.appContext);
        slidesMotor = hardwareMap.dcMotor.get("slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Subscribe to slide encoder pub/sub
        stateStore.subscribe(RobotStateKey.SLIDE_POSITION, (key, value) -> {
            telemetry.addLine("Slide updated to: " + value);
        });

        waitForStart();

        while (opModeIsActive()) {
            Pose pose = follower.getPose();

            // Store pose
            stateStore.set(RobotStateKey.POSE_X, pose.getX());
            stateStore.set(RobotStateKey.POSE_Y, pose.getY());
            stateStore.set(RobotStateKey.HEADING, pose.getHeading());

            // Movement state detection
            double dx = pose.getX() - lastPose.getX();
            double dy = pose.getY() - lastPose.getY();
            double distance = Math.sqrt(dx * dx + dy * dy);

            if (distance > POSITION_THRESHOLD) {
                stateStore.setMovementState(SQLiteStateStore.MovementState.MOVING);
            } else if (stateStore.getMovementState() == SQLiteStateStore.MovementState.MOVING) {
                stateStore.setMovementState(SQLiteStateStore.MovementState.JUST_STOPPED);
            } else {
                stateStore.setMovementState(SQLiteStateStore.MovementState.STOPPED);
            }

            lastPose = pose;

            // Slide control via gamepad
            if (gamepad1.dpad_up && targetSlideTicks < SLIDE_MAX) {
                targetSlideTicks += 20;
            } else if (gamepad1.dpad_down && targetSlideTicks > SLIDE_MIN) {
                targetSlideTicks -= 20;
            }

            int currentTicks = slidesMotor.getCurrentPosition();
            double power = 0;

            if (Math.abs(targetSlideTicks - currentTicks) > 10) {
                if (targetSlideTicks > currentTicks) {
                    power = 0.5;
                    stateStore.setSlideState(SQLiteStateStore.SlideState.EXTENDING);
                } else {
                    power = -0.5;
                    stateStore.setSlideState(SQLiteStateStore.SlideState.RETRACTING);
                }
            } else {
                power = 0;
                if (currentTicks >= SLIDE_MAX - 10) {
                    stateStore.setSlideState(SQLiteStateStore.SlideState.EXTENDED);
                } else if (currentTicks <= SLIDE_MIN + 10) {
                    stateStore.setSlideState(SQLiteStateStore.SlideState.RETRACTED);
                }
            }

            slidesMotor.setPower(power);
            stateStore.set(RobotStateKey.SLIDE_POSITION, currentTicks);

            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Slide", currentTicks);
            telemetry.addData("SlideState", stateStore.getSlideState());
            telemetry.addData("MoveState", stateStore.getMovementState());
            telemetry.update();

            follower.update();
            sleep(100);
        }

        stateStore.close();
    }
}
