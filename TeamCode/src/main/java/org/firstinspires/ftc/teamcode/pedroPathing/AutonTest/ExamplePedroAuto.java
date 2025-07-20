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

@Autonomous(name = "Forward and Back Auto", group = "Examples")
public class ExamplePedroAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(0, 0, 0); // Facing right (0 radians)
    private final Pose forwardPose = new Pose(10, 0, 0); // 10 inches forward in x

    private Path forwardPath, backwardPath;

    public void buildPaths() {
        forwardPath = new Path(new BezierLine(new Pose(), new Pose()));
        forwardPath.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());

        backwardPath = new Path(new BezierLine(new Pose(), new Pose()));
        backwardPath.setLinearHeadingInterpolation(forwardPose.getHeading(), startPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(forwardPath);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(backwardPath);
                    setPathState(2);
                }
                break;
            case 2:
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
