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
    private final Pose blueGrabSamplePose2 = centerCoordsPose(72, 44, 1.5708); // go to submersible to pick up sample
    private final Pose blueBasketPose = centerCoordsPose(18, 126, 2.0944); // go to basket
    private final Pose blueObservPose = centerCoordsPose(24, 20, 3.14159); // go to observation zone to drop sample/pick up specimen

    // all red poses
    private final Pose redStartPose1 = centerCoordsPose(136, 64, 3.14159); // Facing left (pi radians)
    private final Pose redStartPose2 = centerCoordsPose(136, 80, 3.14159);// Facing left (pi radians)
    private final Pose redScoreSpecimenPose1 = centerCoordsPose(108, 64, 3.14159);
    private final Pose redScoreSpecimenPose2 = centerCoordsPose(108, 80, 3.14159);
    private final Pose redGrabSamplePose1 = centerCoordsPose(108, 44, 1.5708); // go to submersible to pick up sample
    private final Pose redGrabSamplePose2 = centerCoordsPose(72, 100, 4.71239);
    private final Pose redBasketPose = centerCoordsPose(126, 18, 5.49779); // go to basket
    private final Pose redObservPose = centerCoordsPose(120, 124, 0);


    private Path blueScoreSpecimen1, blueGrabSampleTop1, blueGrabSampleTop2, blueGrabSampleTop, blueBasketToSubmers, blueSubmersToBasket;
    private Path blueScoreSpecimen2, blueGrabSampleBottom1, blueGrabSampleBottom2, blueGrabSampleBottom, blueSubmersToObserv, blueObservToSubmers, blueScoreSpecimen;
    private Path redScoreSpecimen1, redGrabSampleBottom1, redGrabSampleBottom2, redGrabSampleBottom, redBasketToSubmers, redSubmersToBasket;
    private Path redScoreSpecimen2, redGrabSampleTop1, redGrabSampleTop2, redGrabSampleTop, redSubmersToObserv, redObservToSubmers, redScoreSpecimen;

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

        // all blue 2 paths (further from basket)
        // score specimen
        blueScoreSpecimen2 = new Path(new BezierLine(blueStartPose2, blueScoreSpecimenPose2));
        blueScoreSpecimen2.setLinearHeadingInterpolation(blueStartPose2.getHeading(), blueScoreSpecimenPose2.getHeading());
        // go down (part 1 of getting sample)
        blueGrabSampleBottom1 = new Path(new BezierLine(blueScoreSpecimenPose2, centerCoordsPose(36, 34, 0)));
        blueGrabSampleBottom1.setLinearHeadingInterpolation(blueScoreSpecimenPose2.getHeading(), centerCoordsPose(36, 34, 0).getHeading());
        // go right (part 2 of getting sample)
        blueGrabSampleBottom2 = new Path(new BezierLine(centerCoordsPose(36, 34, 0), centerCoordsPose(72, 34, 0)));
        blueGrabSampleBottom2.setLinearHeadingInterpolation(centerCoordsPose(36, 34, 0).getHeading(), centerCoordsPose(72, 34, 0).getHeading());
        // go up towards submersible (part 3 of getting sample)
        blueGrabSampleBottom = new Path(new BezierLine(centerCoordsPose(72, 34, 0), blueGrabSamplePose2));
        blueGrabSampleBottom.setLinearHeadingInterpolation(centerCoordsPose(72, 34, 0).getHeading(), blueGrabSamplePose2.getHeading());
        // drop sample in observation zone
        blueSubmersToObserv = new Path(new BezierLine(blueGrabSamplePose2, blueObservPose));
        blueSubmersToObserv.setLinearHeadingInterpolation(blueGrabSamplePose2.getHeading(), blueObservPose.getHeading());
        // take specimen from observation and score
        blueScoreSpecimen = new Path(new BezierLine(blueObservPose, blueScoreSpecimenPose2));
        blueScoreSpecimen.setLinearHeadingInterpolation(blueObservPose.getHeading(), blueScoreSpecimenPose2.getHeading());

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

        // all red 2 paths (further from basket)
        // score specimen
        redScoreSpecimen2 = new Path(new BezierLine(redStartPose2, redScoreSpecimenPose2));
        redScoreSpecimen2.setLinearHeadingInterpolation(redStartPose2.getHeading(), redScoreSpecimenPose2.getHeading());
        // go down (part 1 of getting sample)
        redGrabSampleTop1 = new Path(new BezierLine(redScoreSpecimenPose2, centerCoordsPose(108, 110, 0)));
        redGrabSampleTop1.setLinearHeadingInterpolation(redScoreSpecimenPose2.getHeading(), centerCoordsPose(108, 110, 0).getHeading());
        // go right (part 2 of getting sample)
        redGrabSampleTop2 = new Path(new BezierLine(centerCoordsPose(108, 110, 0), centerCoordsPose(72, 110, 0)));
        redGrabSampleTop2.setLinearHeadingInterpolation(centerCoordsPose(108, 110, 0).getHeading(), centerCoordsPose(72, 110, 0).getHeading());
        // go up towards submersible (part 3 of getting sample)
        redGrabSampleTop = new Path(new BezierLine(centerCoordsPose(72, 110, 0), redGrabSamplePose2));
        redGrabSampleTop.setLinearHeadingInterpolation(centerCoordsPose(72, 110, 0).getHeading(), redGrabSamplePose2.getHeading());
        // drop sample in observation zone
        redSubmersToObserv = new Path(new BezierLine(redGrabSamplePose2, redObservPose));
        redSubmersToObserv.setLinearHeadingInterpolation(redGrabSamplePose2.getHeading(), redObservPose.getHeading());
        // take specimen from observation and score
        redScoreSpecimen = new Path(new BezierLine(redObservPose, redScoreSpecimenPose2));
        redScoreSpecimen.setLinearHeadingInterpolation(redObservPose.getHeading(), redScoreSpecimenPose2.getHeading());


    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // blue position 1 (closer to basket)
                follower.followPath(blueScoreSpecimen1);
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
                follower.followPath(blueScoreSpecimen2);
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

                // red position 1 (closer to basket)
                follower.followPath(redScoreSpecimen1);
                follower.followPath(redGrabSampleBottom1);
                follower.followPath(redGrabSampleBottom2);
                follower.followPath(redGrabSampleBottom);
                // extend slides
                // align and pick up sample
                follower.followPath(redSubmersToBasket);
                // score sample in basket
                follower.followPath(redBasketToSubmers);
                // extend slides
                // align and pick up sample
                follower.followPath(redSubmersToBasket);
                // score sample in basket
                follower.followPath(redBasketToSubmers);
                // extend slides
                // align and pick up sample
                follower.followPath(redSubmersToBasket);
                // score sample in basket
                follower.followPath(redBasketToSubmers);
                // extend slides
                // align and pick up sample
                follower.followPath(redSubmersToBasket);
                // score sample in basket

                /**
                if (!follower.isBusy()) {
                    setPathState(-1); // End
                }
                break;
                 **/
            case 3:
                // blue position 1 (closer to basket)
                follower.followPath(redScoreSpecimen2);
                follower.followPath(redGrabSampleBottom1);
                follower.followPath(redGrabSampleBottom2);
                follower.followPath(redGrabSampleBottom);
                // extend slides
                // align and pick up sample
                follower.followPath(redSubmersToObserv);
                // drop sample in observation zone
                // wait for human player 5 seconds then pick up specimen
                follower.followPath(redScoreSpecimen);
                // score specimen on bar
                follower.followPath(redGrabSampleBottom1);
                follower.followPath(redGrabSampleBottom2);
                follower.followPath(redGrabSampleBottom);
                // extend slides
                // align and pick up sample
                follower.followPath(redSubmersToObserv);
                // drop sample in observation zone
                // wait for human player 5 seconds then pick up specimen
                follower.followPath(redScoreSpecimen);
                // score specimen on bar
                follower.followPath(redGrabSampleBottom1);
                follower.followPath(redGrabSampleBottom2);
                follower.followPath(redGrabSampleBottom);
                // extend slides
                // align and pick up sample
                follower.followPath(redSubmersToObserv);
                // drop sample in observation zone
                // wait for human player 5 seconds then pick up specimen
                follower.followPath(redScoreSpecimen);
                // score specimen on bar
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
