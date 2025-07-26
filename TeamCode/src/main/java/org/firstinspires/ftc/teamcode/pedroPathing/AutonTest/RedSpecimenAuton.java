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

@Autonomous(name = "RedSpecimenAuton", group = "Autonomous")
public class RedSpecimenAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    /** Convert Pedro coords to center-origin coords */
    private Pose centerCoordsPose(double xPedro, double yPedro, double headingRad) {
        return new Pose(xPedro - 72, yPedro - 72, headingRad);
    }

    // all red poses
    private final Pose redStartPose2 = centerCoordsPose(136, 80, 3.14159);// Facing left (pi radians)
    private final Pose redScoreSpecimenPose2 = centerCoordsPose(108, 80, 3.14159);
    private final Pose redGrabSamplePose2 = centerCoordsPose(72, 100, 4.71239);
    private final Pose redBasketPose = centerCoordsPose(126, 18, 5.49779); // go to basket
    private final Pose redObservPose = centerCoordsPose(120, 124, 0);


    private Path redScoreSpecimen2, redGrabSampleTop1, redGrabSampleTop2, redGrabSampleTop, redSubmersToObserv, redObservToSubmers, redScoreSpecimen;

    public void buildPaths() {
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
                follower.followPath(redScoreSpecimen2);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop2);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop);
                    // extend slides
                    // align and pick up sample
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(redSubmersToObserv);
                    // drop sample in observation zone
                    // wait for human player 5 seconds then pick up specimen
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(redScoreSpecimen2);
                    // score specimen on bar
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop1);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop2);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop);
                    // extend slides
                    // align and pick up sample
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(redSubmersToObserv);
                    // drop sample in observation zone
                    // wait for human player 5 seconds then pick up specimen
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(redScoreSpecimen2);
                    // score specimen on bar
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop1);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop2);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleTop);
                    // extend slides
                    // align and pick up sample
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(redSubmersToObserv);
                    // drop sample in observation zone
                    // wait for human player 5 seconds then pick up specimen
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(redScoreSpecimen2);
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
        follower.setStartingPose(redStartPose2);

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
