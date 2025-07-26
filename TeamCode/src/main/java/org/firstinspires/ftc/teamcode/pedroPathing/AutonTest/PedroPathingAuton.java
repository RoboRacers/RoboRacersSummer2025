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

@Autonomous(name = "auton", group = "auton")
public class PedroPathingAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    /** Convert Pedro coords to center-origin coords */
    private Pose centerCoordsPose(double xPedro, double yPedro, double headingRad) {
        return new Pose(xPedro - 72, yPedro - 72, headingRad);
    }

    // all blue poses
    private final Pose blueStartPose1 = centerCoordsPose(8, 80, 0); // Facing right (0 radians)
    private final Pose blueStartPose2 = centerCoordsPose(8, 64, 0); // Facing right (0 radians)
    private final Pose blueScoreSpecimenPose1 = centerCoordsPose(36, 80, 0);
    private final Pose blueScoreSpecimenPose2 = centerCoordsPose(36, 64, 0);
    private final Pose blueGrabSamplePose1 = centerCoordsPose(72, 100, 4.71239); // go to submersible to pick up sample
    private final Pose blueGrabSamplePose2 = centerCoordsPose(72, 44, 4.71239); // go to submersible to pick up sample
    private final Pose blueBasketPose = centerCoordsPose(18, 126, 2.0944); // go to basket
    private final Pose blueObservPose = centerCoordsPose(24, 20, 3.14159); // go to observation zone to drop sample/pick up specimen

    // all red poses
    private final Pose redStartPose2 = centerCoordsPose(136, 64, 3.14159); // Facing left (pi radians)
    private final Pose redBasket = centerCoordsPose(126, 18, 4.71239); // go to basket
    private final Pose redSubmers = centerCoordsPose(72, 44, 2.0944);
    private final Pose redObservPose = centerCoordsPose(120, 124, 4.71239);


    private Path blueScoreSpecimen1, blueGrabSampleTop1, blueGrabSampleTop2, blueGrabSampleTop, blueBasketToSubmers, blueSubmersToBasket;
    private Path blueScoreSpecimen2, blueGrabSampleBottom1, blueGrabSampleBottom2, blueGrabSampleBottom, blueSubmersToObserv, blueObservToSubmers, blueScoreSpecimen;

    public void buildPaths() {
        // all blue 1 paths
        // score specimen
        blueScoreSpecimen1 = new Path(new BezierLine(blueStartPose1, blueScoreSpecimenPose1));
        blueScoreSpecimen1.setLinearHeadingInterpolation(blueStartPose1.getHeading(), blueScoreSpecimenPose1.getHeading());
        // go up (part 1 of getting sample)
        blueGrabSampleTop1 = new Path(new BezierLine(blueScoreSpecimenPose1, centerCoordsPose(36,110,0)));
        blueGrabSampleTop1.setLinearHeadingInterpolation(blueScoreSpecimenPose1.getHeading(), centerCoordsPose(36,110,0).getHeading());
        // go right (part 2 of getting sample)
        blueGrabSampleTop2 = new Path(new BezierLine(centerCoordsPose(36,110,0), centerCoordsPose(72,110,0)));
        blueGrabSampleTop2.setLinearHeadingInterpolation(centerCoordsPose(36,110,0).getHeading(), centerCoordsPose(72,110,0).getHeading());
        // go down towards submersible (part 3 of getting sample)
        blueGrabSampleTop = new Path(new BezierLine(centerCoordsPose(72,110,0), blueGrabSamplePose1));
        blueGrabSampleTop.setLinearHeadingInterpolation(centerCoordsPose(72,110,0).getHeading(), blueGrabSamplePose1.getHeading());
        // score sample
        blueSubmersToBasket = new Path(new BezierLine(blueGrabSamplePose1, blueBasketPose));
        blueSubmersToBasket.setLinearHeadingInterpolation(blueGrabSamplePose1.getHeading(), blueBasketPose.getHeading());
        // get sample
        blueBasketToSubmers = new Path(new BezierLine(blueBasketPose, blueGrabSamplePose1));
        blueBasketToSubmers.setLinearHeadingInterpolation(blueBasketPose.getHeading(), blueGrabSamplePose1.getHeading());

        // all blue 2 paths
        // score specimen
        blueScoreSpecimen2 = new Path(new BezierLine(blueStartPose2, blueScoreSpecimenPose2));
        blueScoreSpecimen2.setLinearHeadingInterpolation(blueStartPose2.getHeading(), blueScoreSpecimenPose2.getHeading());
        // go down (part 1 of getting sample)
        blueGrabSampleBottom1 = new Path(new BezierLine(blueScoreSpecimenPose2, centerCoordsPose(36,34,0)));
        blueGrabSampleBottom1.setLinearHeadingInterpolation(blueScoreSpecimenPose2.getHeading(), centerCoordsPose(36,34,0).getHeading());
        // go right (part 2 of getting sample)
        blueGrabSampleBottom2 = new Path(new BezierLine(centerCoordsPose(36,34,0), centerCoordsPose(72,34,0)));
        blueGrabSampleBottom2.setLinearHeadingInterpolation(centerCoordsPose(36,34,0).getHeading(), centerCoordsPose(72,34,0).getHeading());
        // go up towards submersible (part 3 of getting sample)
        blueGrabSampleBottom = new Path(new BezierLine(centerCoordsPose(72,34,0), blueGrabSamplePose2));
        blueGrabSampleBottom.setLinearHeadingInterpolation(centerCoordsPose(72,34,0).getHeading(), blueGrabSamplePose2.getHeading());
        // drop sample in observation zone
        blueSubmersToObserv = new Path(new BezierLine(blueGrabSamplePose2, blueObservPose));
        blueSubmersToObserv.setLinearHeadingInterpolation(blueGrabSamplePose2.getHeading(), blueObservPose.getHeading());
        // take specimen from observation and score
        blueScoreSpecimen = new Path(new BezierLine(blueObservPose, blueScoreSpecimenPose2));
        blueScoreSpecimen.setLinearHeadingInterpolation(blueObservPose.getHeading(), blueScoreSpecimenPose2.getHeading());



        /**
        // all blue 2 paths
        blue2ForwardPath = new Path(new BezierLine(blueStartPose2, centerCoordsPose(18, 64, 0)));
        blue2ForwardPath.setLinearHeadingInterpolation(blueStartPose2.getHeading(), centerCoordsPose(18, 64, 0).getHeading());
        blue2BasketPath = new Path(new BezierLine(centerCoordsPose(18, 64, 0), goBlueBasket));
        blue2BasketPath.setLinearHeadingInterpolation(centerCoordsPose(18, 64, 0).getHeading(), goBlueBasket.getHeading());


        // all red 1 paths

        blue1ForwardPath = new Path(new BezierLine(blueStartPose1, centerCoordsPose(18, 80, 0)));
        blue1ForwardPath.setLinearHeadingInterpolation(blueStartPose1.getHeading(), centerCoordsPose(18, 80, 0).getHeading());
        blue1BasketPath = new Path(new BezierLine(centerCoordsPose(18, 80, 0), goBlueBasket));
        blue1BasketPath.setLinearHeadingInterpolation(centerCoordsPose(18, 80, 0).getHeading(), goBlueBasket.getHeading());


        // all red 2 paths (closer to basket)
        red2ForwardPath = new Path(new BezierLine(blueStartPose2, centerCoordsPose(18, 64, 0)));
        red2ForwardPath.setLinearHeadingInterpolation(blueStartPose2.getHeading(), centerCoordsPose(18, 64, 0).getHeading());
        red2BasketPath = new Path(new BezierLine(centerCoordsPose(18, 64, 0), goBlueBasket));
        red2BasketPath.setLinearHeadingInterpolation(centerCoordsPose(18, 64, 0).getHeading(), goBlueBasket.getHeading());

        blueBasketToSubmers = new Path(new BezierLine(goBlueBasket, goBlueSubmers));
        blueBasketToSubmers.setLinearHeadingInterpolation(goBlueBasket.getHeading(), goBlueSubmers.getHeading());
        blueSubmersToBasket = new Path(new BezierLine(goBlueSubmers, goBlueBasket));
        blueSubmersToBasket.setLinearHeadingInterpolation(goBlueSubmers.getHeading(), goBlueBasket.getHeading());



        backwardPath = new Path(new BezierLine(new Pose(), new Pose()));
        backwardPath.setLinearHeadingInterpolation(forwardPose.getHeading(), startPose.getHeading());
         **/
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // blue position 1 (closer to basket)
                follower.followPath(blueScoreSpecimen);
                follower.followPath(blueGrabSampleTop1);
                follower.followPath(blueGrabSampleTop2);
                follower.followPath(blueGrabSampleTop);
                // extend slides
                // align and pick up sample
                follower.followPath(blueSubmersToBasket);
                // score sample in basket
                follower.followPath(blueBasketToSubmers);
                // extend slides
                // align and pick up sample
                follower.followPath(blueSubmersToBasket);
                // score sample in basket
                follower.followPath(blueBasketToSubmers);
                // extend slides
                // align and pick up sample
                follower.followPath(blueSubmersToBasket);
                // score sample in basket
                follower.followPath(blueBasketToSubmers);
                // extend slides
                // align and pick up sample
                follower.followPath(blueSubmersToBasket);
                // score sample in basket

                // setPathState(1);
                break;
            case 1:
                // blue position 1 (closer to basket)
                follower.followPath(blueScoreSpecimen);
                follower.followPath(blueGrabSampleBottom1);
                follower.followPath(blueGrabSampleBottom2);
                follower.followPath(blueGrabSampleBottom);
                // extend slides
                // align and pick up sample
                follower.followPath(blueSubmersToObserv);
                // drop sample in observation zone
                // wait for human player 5 seconds then pick up specimen
                follower.followPath(blueScoreSpecimen);
                // score specimen on bar
                follower.followPath(blueGrabSampleBottom1);
                follower.followPath(blueGrabSampleBottom2);
                follower.followPath(blueGrabSampleBottom);
                // extend slides
                // align and pick up sample
                follower.followPath(blueSubmersToObserv);
                // drop sample in observation zone
                // wait for human player 5 seconds then pick up specimen
                follower.followPath(blueScoreSpecimen);
                // score specimen on bar
                follower.followPath(blueGrabSampleBottom1);
                follower.followPath(blueGrabSampleBottom2);
                follower.followPath(blueGrabSampleBottom);
                // extend slides
                // align and pick up sample
                follower.followPath(blueSubmersToObserv);
                // drop sample in observation zone
                // wait for human player 5 seconds then pick up specimen
                follower.followPath(blueScoreSpecimen);
                // score specimen on bar

                /**
                if (!follower.isBusy()) {
                    follower.followPath(backwardPath);
                    setPathState(2);
                }
                 **/
                break;
            case 2:
                /**
                // red position 2 (closer to basket)
                follower.followPath(blue2ForwardPath);
                follower.followPath(blue2BasketPath);
                // rotate to 120 degrees to face basket
                // score sample in basket
                follower.followPath(blueBasketToSubmers);
                // rotate to 270 degrees to face submersible
                // pick up sample from submersible
                follower.followPath(blueSubmersToBasket);
                // rotate to 120 degrees to face basket
                // score sample in basket
                // repeat for as long as auton runs

                /**
                if (!follower.isBusy()) {
                    setPathState(-1); // End
                }
                break;
                 **/
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
