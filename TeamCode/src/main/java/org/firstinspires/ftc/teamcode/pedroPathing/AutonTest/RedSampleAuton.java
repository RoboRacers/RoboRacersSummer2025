package org.firstinspires.ftc.teamcode.pedroPathing.AutonTest;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "RedSampleAuton", group = "Autonomous")
public class RedSampleAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    /** Convert Pedro coords to center-origin coords */
    private Pose centerCoordsPose(double xPedro, double yPedro, double headingRad) {
        return new Pose(xPedro - 72, yPedro - 72, headingRad);
    }

    private final Pose redStartPose1 = centerCoordsPose(136, 64, 3.14159); // Facing left (pi radians)
    private final Pose redScoreSpecimenPose1 = centerCoordsPose(108, 64, 3.14159);
    private final Pose redGrabSamplePose1 = centerCoordsPose(108, 44, 1.5708); // go to submersible to pick up sample
    private final Pose redBasketPose = centerCoordsPose(126, 18, 5.49779); // go to basket
    private final Pose redObservPose = centerCoordsPose(120, 124, 0);


    private Path redScoreSpecimen1, redGrabSampleBottom1, redGrabSampleBottom2, redGrabSampleBottom, redBasketToSubmers, redSubmersToBasket;

    public void buildPaths() {
        // all red 1 paths (closer to basket)
        // score specimen
        redScoreSpecimen1 = new Path(new BezierLine(redStartPose1, redScoreSpecimenPose1));
        redScoreSpecimen1.setLinearHeadingInterpolation(redStartPose1.getHeading(), redScoreSpecimenPose1.getHeading());
        // go up (part 1 of getting sample)
        redGrabSampleBottom1 = new Path(new BezierLine(redScoreSpecimenPose1, centerCoordsPose(108, 34, 0)));
        redGrabSampleBottom1.setLinearHeadingInterpolation(redScoreSpecimenPose1.getHeading(), centerCoordsPose(108, 34, 0).getHeading());
        // go right (part 2 of getting sample)
        redGrabSampleBottom2 = new Path(new BezierLine(centerCoordsPose(108, 34, 0), centerCoordsPose(72, 34, 0)));
        redGrabSampleBottom2.setLinearHeadingInterpolation(centerCoordsPose(108, 34, 0).getHeading(), centerCoordsPose(72, 34, 0).getHeading());
        // go down towards submersible (part 3 of getting sample)
        redGrabSampleBottom = new Path(new BezierLine(centerCoordsPose(72, 34, 0), redGrabSamplePose1));
        redGrabSampleBottom.setLinearHeadingInterpolation(centerCoordsPose(72, 34, 0).getHeading(), redGrabSamplePose1.getHeading());
        // score sample
        redSubmersToBasket = new Path(new BezierLine(redGrabSamplePose1, redBasketPose));
        redSubmersToBasket.setLinearHeadingInterpolation(redGrabSamplePose1.getHeading(), redBasketPose.getHeading());
        // get sample
        redBasketToSubmers = new Path(new BezierLine(redBasketPose, redGrabSamplePose1));
        redBasketToSubmers.setLinearHeadingInterpolation(redBasketPose.getHeading(), redGrabSamplePose1.getHeading());

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // red position 1 (closer to basket)
                follower.followPath(redScoreSpecimen1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleBottom1);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(redGrabSampleBottom2);
                    setPathState(3);
                }
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(redGrabSampleBottom);
                    // extend slides
                    // align and pick up sample
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(redSubmersToBasket);
                    // score sample in basket
                    setPathState(5);
                }
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(redBasketToSubmers);
                    // extend slides
                    // align and pick up sample
                    setPathState(6);
                }
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(redSubmersToBasket);
                    // score sample in basket
                    setPathState(7);
                }
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(redBasketToSubmers);
                    // extend slides
                    // align and pick up sample
                    setPathState(8);
                }
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(redSubmersToBasket);
                    // score sample in basket
                    setPathState(9);
                }
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(redBasketToSubmers);
                    // extend slides
                    // align and pick up sample
                    setPathState(9);
                }
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(redSubmersToBasket);
                    // score sample in basket
                    setPathState(10);
                }
            case 11:
                if (!follower.isBusy()) {
                    setPathState(-1); // End
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(redStartPose1);

        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override public void init_loop() {}
    @Override public void stop() {}
}
