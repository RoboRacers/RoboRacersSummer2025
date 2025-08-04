package org.firstinspires.ftc.teamcode.teleop.AutoBasketDriveTeleop;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Auto Drive test", group = "Teleop")
public class PedroTeleOpPinpointV2 extends OpMode {
    private Follower follower;
    public PathChain chain;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    private boolean goingToBasket = false;

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

    private final Pose basketPose = new Pose(-55,55, Math.toRadians(315));

    private Pose endPose = new Pose(0, 0, Math.toRadians(0));


    // Define zone corners (for clarity, minX < maxX, minY < maxY)
    private final double zoneA_minX = -72, zoneA_maxX = -36;
    private final double zoneA_minY = 36, zoneA_maxY = 72;

    private final double zoneB_minX = -24, zoneB_maxX = 24;
    private final double zoneB_minY = 24, zoneB_maxY = 40;


    private boolean isInsideBox(Pose pose, double minX, double maxX, double minY, double maxY) {
        return pose.getX() >= minX && pose.getX() <= maxX &&
                pose.getY() >= minY && pose.getY() <= maxY;
    }


    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(-62, 12, Math.toRadians(0)));

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
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

    }


    @Override
    public void loop() {


        if (goingToBasket) {
            follower.update();
            if (!follower.isBusy() || follower.atPose(endPose, 2, 2) || follower.atPose(basketPose,3,3)) {
                goingToBasket = false;
                follower.breakFollowing();
            }
            telemetry.addLine("Not Here");
            telemetry.update();
        } else {
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    (-gamepad1.right_stick_x * 0.75),
                    true
            );
//            telemetry.update();
//            follower.update();

            if (gamepad1.y) {
                Pose currentPose = follower.getPose();

                if (isInsideBox(currentPose, zoneA_minX, zoneA_maxX, zoneA_minY, zoneA_maxY)) {
                    telemetry.addLine("In Zone A - Placeholder action triggered.");
                    endPose = new Pose(-51.5, 51.5, Math.toRadians(315));

                    Path moveDownPath = new Path(new BezierLine(currentPose, endPose));
                    moveDownPath.setLinearHeadingInterpolation(currentPose.getHeading(), endPose.getHeading());
                    chain = new PathChain(moveDownPath);
                    follower.followPath(chain);
                    goingToBasket = true;

                    // TODO: Insert Zone A action
                } else if (isInsideBox(currentPose, zoneB_minX, zoneB_maxX, zoneB_minY, zoneB_maxY)) {
                    telemetry.addLine("In Zone B - Placeholder action triggered.");
                    endPose = new Pose(currentPose.getX(), 26.25, Math.toRadians(270));

                    Path moveDownPath = new Path(new BezierLine(currentPose, endPose));
                    moveDownPath.setLinearHeadingInterpolation(currentPose.getHeading(), endPose.getHeading());
                    // TODO: Insert Zone B action
                    chain = new PathChain(moveDownPath);
                    follower.followPath(chain);
                    goingToBasket = true;

                }
//                else {
//                    telemetry.addLine("Y pressed outside of action zones.");
//                }
            }



            if (gamepad1.square) {
                Pose currentPose = follower.getPose();

                double subLeft = -18;
                double subRight = 18;
                double subTop = 18;
                double subBottom = -18;

                boolean intersectsSub = doesLineCrossRectangle(currentPose, basketPose, subLeft, subRight, subBottom, subTop);

                Path finalPath;

                if (intersectsSub) {
                    Pose midPose = (currentPose.getY() > 0)
                            ? new Pose(0, subTop + 10, 0)
                            : new Pose(0, subBottom - 10, 0);

                    Path firstPath = new Path(new BezierLine(currentPose, midPose));
                    firstPath.setLinearHeadingInterpolation(currentPose.getHeading(), midPose.getHeading());

                    Path secondPath = new Path(new BezierLine(midPose, basketPose));
                    secondPath.setLinearHeadingInterpolation(midPose.getHeading(), basketPose.getHeading());

                    chain = new PathChain(firstPath, secondPath);

                } else {
                    finalPath = new Path(new BezierLine(currentPose, basketPose));
                    finalPath.setLinearHeadingInterpolation(currentPose.getHeading(), basketPose.getHeading());

                    chain = new PathChain(finalPath); // ✅ This line was missing
                }

                follower.followPath(chain);
                goingToBasket = true;
            }
        }
//        follower.setTeleOpMovementVectors(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                (-gamepad1.right_stick_x * 0.75),
//                true
//        );
        telemetry.addLine("Here");
        follower.update();

        telemetry.addData("Mode", goingToBasket ? "Auto Path to Basket" : "Manual Drive");
        Pose pose = follower.getPose();
        telemetry.addData("x", pose.getX());

        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
        telemetry.addData("goingToBasket", goingToBasket);
        telemetry.addData("follower.isBusy()", follower.isBusy());

        telemetry.update();
    }

    private boolean doesLineCrossRectangle(Pose start, Pose end, double left, double right, double bottom, double top) {
        return lineIntersectsBox(start.getX(), start.getY(), end.getX(), end.getY(), left, right, bottom, top);
    }

    private boolean lineIntersectsBox(double x1, double y1, double x2, double y2,
                                      double minX, double maxX, double minY, double maxY) {
        return lineIntersectsLine(x1, y1, x2, y2, minX, minY, maxX, minY) || // bottom
                lineIntersectsLine(x1, y1, x2, y2, minX, maxY, maxX, maxY) || // top
                lineIntersectsLine(x1, y1, x2, y2, minX, minY, minX, maxY) || // left
                lineIntersectsLine(x1, y1, x2, y2, maxX, minY, maxX, maxY);   // right
    }

    private boolean lineIntersectsLine(double x1, double y1, double x2, double y2,
                                       double x3, double y3, double x4, double y4) {
        double denom = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1);
        if (denom == 0.0) return false;

        double ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / denom;
        double ub = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / denom;

        return ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1;
    }
}
