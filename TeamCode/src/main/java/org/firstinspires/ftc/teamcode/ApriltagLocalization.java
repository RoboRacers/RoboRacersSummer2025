package org.firstinspires.ftc.teamcode;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "SUPER COOL GUY", group = "Teleop")
public class ApriltagLocalization extends OpMode {
    private Follower follower;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    private boolean runningPath = false;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Valid AprilTag IDs
    private static final int BLUE_SCORE_BIN_ID = 20;
    private static final int RED_SCORE_BIN_ID = 24;

    // Shared target pose for both red + blue = small triangle (0,0)
    private final Pose triangleTargetPose = new Pose(0, 0, 0);

    // Snapshot storage
    private double capturedYaw = 0;
    private boolean snapshotCaptured = false;

    static {
        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftBack";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightBack";

        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    }

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);

        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void loop() {
        follower.update();

        if (runningPath) {
            if (!follower.isBusy()) {
                runningPath = false;
                telemetry.addLine("Finished auto action");
            }
        } else {
            // Default manual drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lf = (y + x + rx) / denominator;
            double lr = (y - x + rx) / denominator;
            double rf = (y - x - rx) / denominator;
            double rr = (y + x - rx) / denominator;

            leftFront.setPower(lf);
            leftRear.setPower(lr);
            rightFront.setPower(rf);
            rightRear.setPower(rr);

            // === BUTTON COMMANDS ===

            // (1) X button = go to (0,0), then align with Blue AprilTag (20)
            if (gamepad1.x) {
                AprilTagDetection tag = getTagById(BLUE_SCORE_BIN_ID);
                if (tag != null) {
                    goToPoseAndAlign(triangleTargetPose, tag);
                } else {
                    telemetry.addLine("Blue Tag (20) not detected");
                }
            }

            // (2) B button = go to (0,0), then align with Red AprilTag (24)
            if (gamepad1.b) {
                AprilTagDetection tag = getTagById(RED_SCORE_BIN_ID);
                if (tag != null) {
                    goToPoseAndAlign(triangleTargetPose, tag);
                } else {
                    telemetry.addLine("Red Tag (24) not detected");
                }
            }

            // (3) Left Bumper = snapshot Blue AprilTag (20) yaw + position
            if (gamepad1.left_bumper) {
                AprilTagDetection tag = getTagById(BLUE_SCORE_BIN_ID);
                if (tag != null) {
                    capturedYaw = Math.toRadians(tag.ftcPose.yaw);
                    snapshotCaptured = true;
                    telemetry.addLine("Snapshot captured!");
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Tag Distance (in)", "%.2f", tag.ftcPose.range);
                    telemetry.addData("Tag X (in)", "%.2f", tag.ftcPose.x);
                    telemetry.addData("Tag Y (in)", "%.2f", tag.ftcPose.y);
                } else {
                    telemetry.addLine("Blue Tag (20) not detected for snapshot");
                }
            }

            // (4) Y button = drive to (0,0) and align robot to snapshot yaw
            if (gamepad1.y && snapshotCaptured) {
                Pose currentPose = follower.getPose();

                Pose drivePose = new Pose(0, 0, currentPose.getHeading());
                Path drivePath = new Path(new BezierLine(currentPose, drivePose));
                drivePath.setLinearHeadingInterpolation(currentPose.getHeading(), drivePose.getHeading());

                Pose rotatePose = new Pose(0, 0, capturedYaw);
                Path rotatePath = new Path(new BezierLine(drivePose, rotatePose));
                rotatePath.setLinearHeadingInterpolation(drivePose.getHeading(), capturedYaw);

                PathChain chain = new PathChain(drivePath, rotatePath);

                follower.followPath(chain);
                runningPath = true;

                telemetry.addLine("Going to (0,0) and aligning to snapshot yaw");
                telemetry.addData("Captured Yaw (deg)", "%.1f", Math.toDegrees(capturedYaw));
            }
        }

        // Always show robot pose
        Pose pose = follower.getPose();
        telemetry.addData("Mode", runningPath ? "Auto Path" : "Manual Drive");
        telemetry.addData("Robot X", "%.1f", pose.getX());
        telemetry.addData("Robot Y", "%.1f", pose.getY());
        telemetry.addData("Robot Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));

        // Live AprilTag telemetry if any tag is seen
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            telemetry.addData("Seen Tag ID", tag.id);
            telemetry.addData("Tag Distance (in)", "%.2f", tag.ftcPose.range);
            telemetry.addData("Tag X (in)", "%.2f", tag.ftcPose.x);
            telemetry.addData("Tag Y (in)", "%.2f", tag.ftcPose.y);
        } else {
            telemetry.addLine("No tag currently visible");
        }

        telemetry.update();
    }

    /** Get a tag by ID if visible */
    private AprilTagDetection getTagById(int desiredId) {
        for (AprilTagDetection tag : aprilTag.getDetections()) {
            if (tag.id == desiredId) {
                return tag;
            }
        }
        return null;
    }

    /** Full sequence: drive to (0,0), then align heading using AprilTag yaw */
    private void goToPoseAndAlign(Pose targetPose, AprilTagDetection tag) {
        Pose currentPose = follower.getPose();

        // Step 1: path to (0,0) keeping current heading
        Pose drivePose = new Pose(targetPose.getX(), targetPose.getY(), currentPose.getHeading());
        Path drivePath = new Path(new BezierLine(currentPose, drivePose));
        drivePath.setLinearHeadingInterpolation(currentPose.getHeading(), drivePose.getHeading());

        // Step 2: rotate to align with AprilTag
        double desiredHeading = currentPose.getHeading() + Math.toRadians(tag.ftcPose.yaw);
        Pose rotatePose = new Pose(drivePose.getX(), drivePose.getY(), desiredHeading);
        Path rotatePath = new Path(new BezierLine(drivePose, rotatePose));
        rotatePath.setLinearHeadingInterpolation(drivePose.getHeading(), desiredHeading);

        PathChain chain = new PathChain(drivePath, rotatePath);

        follower.followPath(chain);
        runningPath = true;

        telemetry.addData("Driving + Aligning to Tag ID", tag.id);
        telemetry.addData("Final Heading (deg)", "%.1f", Math.toDegrees(desiredHeading));
        telemetry.addData("Tag Distance (in)", "%.2f", tag.ftcPose.range);
    }
}
