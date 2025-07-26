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

@Autonomous(name = "BlueSampleAuton", group = "Autonomous")
public class BlueSampleAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    /** Convert Pedro coords to center-origin coords */
    private Pose centerCoordsPose(double xPedro, double yPedro, double headingRad) {
        return new Pose(xPedro - 72, yPedro - 72, headingRad);
    }

    private final Pose blueStartPose1 = centerCoordsPose(8, 80, 0); // Facing right (0 radians)
    private final Pose blueScoreSpecimenPose1 = centerCoordsPose(36, 80, 0);
    private final Pose blueGrabSamplePose1 = centerCoordsPose(72, 100, 4.71239); // go to submersible to pick up sample
    private final Pose blueBasketPose = centerCoordsPose(18, 126, 2.0944); // go to basket
    private final Pose blueObservPose = centerCoordsPose(24, 20, 3.14159); // go to observation zone to drop sample/pick up specimen


    private Path blueScoreSpecimen1, blueGrabSampleTop1, blueGrabSampleTop2, blueGrabSampleTop, blueBasketToSubmers, blueSubmersToBasket;

    public void buildPaths() {
        // all blue 1 paths (closer to basket)
        // score specimen
        blueScoreSpecimen1 = new Path(new BezierLine(blueStartPose1, blueScoreSpecimenPose1));
        blueScoreSpecimen1.setLinearHeadingInterpolation(blueStartPose1.getHeading(), blueScoreSpecimenPose1.getHeading());
        // go up (part 1 of getting sample)
        blueGrabSampleTop1 = new Path(new BezierLine(blueScoreSpecimenPose1, centerCoordsPose(36, 110, 0)));
        blueGrabSampleTop1.setLinearHeadingInterpolation(blueScoreSpecimenPose1.getHeading(), centerCoordsPose(36, 110, 0).getHeading());
        // go right (part 2 of getting sample)
        blueGrabSampleTop2 = new Path(new BezierLine(centerCoordsPose(36, 110, 0), centerCoordsPose(72, 110, 0)));
        blueGrabSampleTop2.setLinearHeadingInterpolation(centerCoordsPose(36, 110, 0).getHeading(), centerCoordsPose(72, 110, 0).getHeading());
        // go down towards submersible (part 3 of getting sample)
        blueGrabSampleTop = new Path(new BezierLine(centerCoordsPose(72, 110, 0), blueGrabSamplePose1));
        blueGrabSampleTop.setLinearHeadingInterpolation(centerCoordsPose(72, 110, 0).getHeading(), blueGrabSamplePose1.getHeading());
        // score sample
        blueSubmersToBasket = new Path(new BezierLine(blueGrabSamplePose1, blueBasketPose));
        blueSubmersToBasket.setLinearHeadingInterpolation(blueGrabSamplePose1.getHeading(), blueBasketPose.getHeading());
        // get sample
        blueBasketToSubmers = new Path(new BezierLine(blueBasketPose, blueGrabSamplePose1));
        blueBasketToSubmers.setLinearHeadingInterpolation(blueBasketPose.getHeading(), blueGrabSamplePose1.getHeading());

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // red position 1 (closer to basket)
                follower.followPath(blueScoreSpecimen1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleTop1);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(blueGrabSampleTop2);
                    setPathState(3);
                }
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(blueGrabSampleTop);
                    // extend slides
                    // align and pick up sample
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(blueSubmersToBasket);
                    // score sample in basket
                    setPathState(5);
                }
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(blueBasketToSubmers);
                    // extend slides
                    // align and pick up sample
                    setPathState(6);
                }
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(blueSubmersToBasket);
                    // score sample in basket
                    setPathState(7);
                }
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(blueBasketToSubmers);
                    // extend slides
                    // align and pick up sample
                    setPathState(8);
                }
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(blueSubmersToBasket);
                    // score sample in basket
                    setPathState(9);
                }
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(blueBasketToSubmers);
                    // extend slides
                    // align and pick up sample
                    setPathState(9);
                }
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(blueSubmersToBasket);
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
        follower.setStartingPose(blueStartPose1);

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
