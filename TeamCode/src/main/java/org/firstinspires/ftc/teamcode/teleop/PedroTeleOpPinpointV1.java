package org.firstinspires.ftc.teamcode.teleop;

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
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Pedro Go To Basket", group = "Teleop")
public class PedroTeleOpPinpointV1 extends OpMode {
    private Follower follower;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    private boolean goingToBasket = false;
    private boolean basketAfterDetour = false;
    private Path toBasketPath;
    private Pose nextTargetPose;

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

    private final Pose basketPose = new Pose(54, 54, Math.toRadians(45));

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(19, 71, Math.toRadians(0)));

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

    private boolean crossesSubmersible(Pose start, Pose end) {
        double subMinX = -24, subMaxX = 24;
        double subMinY = -24, subMaxY = 24;
        return lineIntersectsBox(start.getX(), start.getY(), end.getX(), end.getY(),
                subMinX, subMinY, subMaxX, subMaxY);
    }

    private boolean lineIntersectsBox(double x1, double y1, double x2, double y2,
                                      double rx1, double ry1, double rx2, double ry2) {
        return lineIntersectsLine(x1, y1, x2, y2, rx1, ry1, rx1, ry2) ||
                lineIntersectsLine(x1, y1, x2, y2, rx2, ry1, rx2, ry2) ||
                lineIntersectsLine(x1, y1, x2, y2, rx1, ry1, rx2, ry1) ||
                lineIntersectsLine(x1, y1, x2, y2, rx1, ry2, rx2, ry2);
    }

    private boolean lineIntersectsLine(double x1, double y1, double x2, double y2,
                                       double x3, double y3, double x4, double y4) {
        double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
        if (denom == 0) return false;

        double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
        double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

        return ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1;
    }

    @Override
    public void loop() {
        follower.update();

        if (goingToBasket) {
            if (!follower.isBusy()) {
                if (basketAfterDetour) {
                    BezierLine finalLine = new BezierLine(follower.getPose(), nextTargetPose);
                    toBasketPath = new Path(finalLine);
                    follower.followPath(toBasketPath);
                    basketAfterDetour = false;
                } else {
                    goingToBasket = false;
                }
            }
        } else {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) > 0.05 || Math.abs(x) > 0.05 || Math.abs(rx) > 0.05) {
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double lf = (y + x + rx) / denominator;
                double lr = (y - x + rx) / denominator;
                double rf = (y - x - rx) / denominator;
                double rr = (y + x - rx) / denominator;

                leftFront.setPower(lf);
                leftRear.setPower(lr);
                rightFront.setPower(rf);
                rightRear.setPower(rr);
            } else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }

            if (gamepad1.square) {
                Pose currentPose = follower.getPose();

                // Clamp basket pose to field bounds
                double bx = Math.max(-72, Math.min(72, basketPose.getX()));
                double by = Math.max(-72, Math.min(72, basketPose.getY()));
                Pose safeBasketPose = new Pose(bx, by, basketPose.getHeading());

                boolean crosses = crossesSubmersible(currentPose, safeBasketPose);

                if (crosses) {
                    // Detour either horizontally or vertically based on current position
                    double detourX = (currentPose.getX() < 0) ? -30 : 30;
                    double detourY = (Math.abs(currentPose.getY()) > Math.abs(currentPose.getX())) ?
                            ((currentPose.getY() < 0) ? -30 : 30) : currentPose.getY();

                    detourX = Math.max(-72, Math.min(72, detourX));
                    detourY = Math.max(-72, Math.min(72, detourY));

                    Pose detourPose = new Pose(detourX, detourY, Math.toRadians(0));
                    BezierLine toDetour = new BezierLine(currentPose, detourPose);
                    toBasketPath = new Path(toDetour);

                    nextTargetPose = safeBasketPose;
                    basketAfterDetour = true;
                    goingToBasket = true;
                    follower.followPath(toBasketPath);

                } else {
                    BezierLine direct = new BezierLine(currentPose, safeBasketPose);
                    toBasketPath = new Path(direct);
                    follower.followPath(toBasketPath);
                    goingToBasket = true;
                }
            }
        }

        telemetry.addData("Mode", goingToBasket ? "Auto Path to Basket" : "Manual Drive");
        Pose pose = follower.getPose();
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", pose.getHeading());
        telemetry.update();
    }
}
