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
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
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
    private PathChain toBasketPath;

    static{
        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftBack";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightBack";

        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    }

    private final Pose basketPose = new Pose(4, -22, Math.toRadians(45));

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        // Set robot's actual starting location (update this to match your starting position)
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
            motor.setPower(0); // Ensure no power at startup
        }
    }

    @Override
    public void loop() {
        // Always update pose for live localization
        follower.update();

        if (goingToBasket) {
            if (!follower.isBusy()) {
                goingToBasket = false;
            }
        } else {
            // Manual drive control with deadzone
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
                // Stop motors if no input
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }

            if (gamepad1.square) {
                Pose currentPose = follower.getPose();
                if (currentPose.getX() > 24 && currentPose.getY() < 24){
                    Pose interPose = new Pose(40,40,Math.toRadians(270));
                    toBasketPath = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(currentPose), new Point(interPose)))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), interPose.getHeading())
                            .addPath(new BezierLine(new Point(interPose), new Point(basketPose)))
                            .setLinearHeadingInterpolation(interPose.getHeading(), basketPose.getHeading())
                            .build();

                    follower.followPath(toBasketPath);
                }
                else if(currentPose.getX()> -24 && currentPose.getY()< -24){
                    Pose interPose = new Pose(-40,-40,Math.toRadians(0));
                    toBasketPath = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(currentPose), new Point(interPose)))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), interPose.getHeading())
                            .addPath(new BezierLine(new Point(interPose), new Point(basketPose)))
                            .setLinearHeadingInterpolation(interPose.getHeading(), basketPose.getHeading())
                            .build();

                    follower.followPath(toBasketPath);
                }
                else{
                    toBasketPath = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(currentPose), new Point(basketPose)))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), basketPose.getHeading())
                            .build();

                    follower.followPath(toBasketPath);
                }
                goingToBasket = true;
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