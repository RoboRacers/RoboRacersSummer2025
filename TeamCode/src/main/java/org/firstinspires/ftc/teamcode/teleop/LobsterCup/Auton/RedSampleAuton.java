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

@Autonomous(name = "Red Sample Auton", group = "Examples")
public class RedSampleAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(12, -68, 270); // Facing right (0 radians)
    private final Pose redScoreSpecimenPose1 = new Pose(5, -26, 270);
    private final Pose redGrabSamplePose1 = new Pose(-49, -38, 270); // go to submersible to pick up sample
    private final Pose redGrabSamplePose2 = new Pose(-59, -38, 270); // go to submersible to pick up sample
    private final Pose redGrabSamplePose3 = new Pose(-69, -38, 270); // go to submersible to pick up sample
    private final Pose redGrabSampleSub = new Pose(22, -11, 188); // go to submersible to pick up sample

    private final Pose redBasketPose = new Pose(-56, -56, 235); // go to basket
    private final Pose redObservPose = new Pose(60, -60, 270); // go to observation zone to drop sample/pick up specimen

    private Path redScoreSpecimenPose1Path,redGrabSamplePose1Path,redGrabSamplePose2Path,redGrabSamplePose3Path,redBasketPosePath,redObservPosePath,redGrabSampleSubPath;

    public void buildPaths() {
        redScoreSpecimenPose1Path = new Path(new BezierLine(new Pose(), new Pose()));
        redScoreSpecimenPose1Path.setLinearHeadingInterpolation(startPose.getHeading(), redScoreSpecimenPose1.getHeading());

        redGrabSamplePose1Path = new Path(new BezierLine(new Pose(), new Pose()));
        redGrabSamplePose1Path.setLinearHeadingInterpolation(redScoreSpecimenPose1.getHeading(), redGrabSamplePose1.getHeading());

        redGrabSamplePose2Path = new Path(new BezierLine(new Pose(), new Pose()));
        redGrabSamplePose2Path.setLinearHeadingInterpolation(redBasketPose.getHeading(), redGrabSamplePose2.getHeading());

        redGrabSamplePose3Path = new Path(new BezierLine(new Pose(), new Pose()));
        redGrabSamplePose3Path.setLinearHeadingInterpolation(redBasketPose.getHeading(), redGrabSamplePose3.getHeading());

        redBasketPosePath = new Path(new BezierLine(new Pose(), new Pose()));
        redBasketPosePath.setLinearHeadingInterpolation(redBasketPose.getHeading(), redGrabSamplePose3.getHeading());

        redGrabSampleSubPath = new Path(new BezierLine(new Pose(), new Pose()));
        redGrabSampleSubPath.setLinearHeadingInterpolation(redBasketPose.getHeading(), redGrabSampleSub.getHeading());

        redObservPosePath = new Path(new BezierLine(new Pose(), new Pose()));
        redObservPosePath.setLinearHeadingInterpolation(redBasketPose.getHeading(), redObservPose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(redScoreSpecimenPose1Path);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSamplePose1Path);
                    setPathState(2);
                }
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(redBasketPosePath);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSamplePose2Path);
                    setPathState(4);
                }
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(redBasketPosePath);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSamplePose3Path);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(redBasketPosePath);
                    setPathState(7);
                }
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(redGrabSampleSubPath);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(redBasketPosePath);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(redObservPosePath);
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
