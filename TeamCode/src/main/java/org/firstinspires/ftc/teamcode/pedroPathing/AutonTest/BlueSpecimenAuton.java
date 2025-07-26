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

@Autonomous(name = "BlueSpecimenAuton", group = "Autonomous")
public class BlueSpecimenAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    /** Convert Pedro coords to center-origin coords */
    private Pose centerCoordsPose(double xPedro, double yPedro, double headingRad) {
        return new Pose(xPedro - 72, yPedro - 72, headingRad);
    }

    private final Pose blueStartPose2 = centerCoordsPose(8, 64, 0); // Facing right (0 radians)
    private final Pose blueScoreSpecimenPose2 = centerCoordsPose(36, 64, 0);
    private final Pose blueGrabSamplePose2 = centerCoordsPose(72, 44, 1.5708); // go to submersible to pick up sample
    private final Pose blueBasketPose = centerCoordsPose(18, 126, 2.0944); // go to basket
    private final Pose blueObservPose = centerCoordsPose(24, 20, 3.14159); // go to observation zone to drop sample/pick up specimen

    private Path blueScoreSpecimen2, blueGrabSampleBottom1, blueGrabSampleBottom2, blueGrabSampleBottom, blueSubmersToObserv, blueObservToSubmers, blueScoreSpecimen;

    public void buildPaths() {
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
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(blueScoreSpecimen2);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom2);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom);
                    // extend slides
                    // align and pick up sample
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(blueSubmersToObserv);
                    // drop sample in observation zone
                    // wait for human player 5 seconds then pick up specimen
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreSpecimen2);
                    // score specimen on bar
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom1);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom2);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom);
                    // extend slides
                    // align and pick up sample
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(blueSubmersToObserv);
                    // drop sample in observation zone
                    // wait for human player 5 seconds then pick up specimen
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreSpecimen2);
                    // score specimen on bar
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom1);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom2);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleBottom);
                    // extend slides
                    // align and pick up sample
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(blueSubmersToObserv);
                    // drop sample in observation zone
                    // wait for human player 5 seconds then pick up specimen
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreSpecimen2);
                    // score specimen on bar
                    setPathState(16);
                }
                break;
            case 16:
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
        follower.setStartingPose(blueStartPose2);

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
