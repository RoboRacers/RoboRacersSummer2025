package org.firstinspires.ftc.teamcode.teleop.LobsterCup.Auton;

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

@Autonomous(name = "Blue Human Player Auton", group = "Examples")
public class BlueHumanPlayerAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(-12, 68, 270); // Facing right (0 radians)
    private final Pose blueScoreSpecimenPose1 = new Pose(5, 26, 270);
    private final Pose blueGrabSamplePose1 = new Pose(-49, 38, 270); // go to submersible to pick up sample
    private final Pose blueGrabSamplePose2 = new Pose(-59, 38, 270); // go to submersible to pick up sample
    private final Pose blueGrabSamplePose3 = new Pose(-69, 38, 270); // go to submersible to pick up sample
    private final Pose blueGrabSampleSub = new Pose(-22, 11, 188); // go to submersible to pick up sample

    private final Pose blueBasketPose = new Pose(56, 56, 235); // go to basket
    private final Pose blueObservPose = new Pose(-60, 60, 270); // go to observation zone to drop sample/pick up specimen

    private Path blueScoreSpecimenPose1Path,blueGrabSamplePose1Path,blueGrabSamplePose2Path,blueGrabSamplePose3Path,blueBasketPosePath,blueObservPosePath,blueGrabSampleSubPath;

    public void buildPaths() {
        blueScoreSpecimenPose1Path = new Path(new BezierLine(new Pose(), new Pose()));
        blueScoreSpecimenPose1Path.setLinearHeadingInterpolation(startPose.getHeading(), blueScoreSpecimenPose1.getHeading());

        blueGrabSamplePose1Path = new Path(new BezierLine(new Pose(), new Pose()));
        blueGrabSamplePose1Path.setLinearHeadingInterpolation(blueScoreSpecimenPose1.getHeading(), blueGrabSamplePose1.getHeading());

        blueGrabSamplePose2Path = new Path(new BezierLine(new Pose(), new Pose()));
        blueGrabSamplePose2Path.setLinearHeadingInterpolation(blueBasketPose.getHeading(), blueGrabSamplePose2.getHeading());

        blueGrabSamplePose3Path = new Path(new BezierLine(new Pose(), new Pose()));
        blueGrabSamplePose3Path.setLinearHeadingInterpolation(blueBasketPose.getHeading(), blueGrabSamplePose3.getHeading());

        blueBasketPosePath = new Path(new BezierLine(new Pose(), new Pose()));
        blueBasketPosePath.setLinearHeadingInterpolation(blueBasketPose.getHeading(), blueGrabSamplePose3.getHeading());

        blueGrabSampleSubPath = new Path(new BezierLine(new Pose(), new Pose()));
        blueGrabSampleSubPath.setLinearHeadingInterpolation(blueBasketPose.getHeading(), blueGrabSampleSub.getHeading());

        blueObservPosePath = new Path(new BezierLine(new Pose(), new Pose()));
        blueObservPosePath.setLinearHeadingInterpolation(blueBasketPose.getHeading(), blueObservPose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(blueScoreSpecimenPose1Path);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSamplePose1Path);
                    setPathState(2);
                }
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(blueObservPosePath);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSamplePose2Path);
                    setPathState(4);
                }
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(blueObservPosePath);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSamplePose3Path);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(blueObservPosePath);
                    setPathState(7);
                }
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(blueGrabSampleSubPath);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(blueBasketPosePath);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(blueObservPosePath);
                    setPathState(10);
                }
            case 10:
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
        follower.setStartingPose(startPose);

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
