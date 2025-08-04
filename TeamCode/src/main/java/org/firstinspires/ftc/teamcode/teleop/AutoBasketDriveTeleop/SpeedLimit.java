package org.firstinspires.ftc.teamcode.teleop.AutoBasketDriveTeleop;

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
import com.pedropathing.util.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "School Zone", group = "Teleop")
public class SpeedLimit extends OpMode {
    private Follower follower;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    static {
        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftBack";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightBack";

        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    }

    // Define the slowdown zones in four corners of the field (2 FTC tiles, 48 inches)
    private final double FIELD_MIN = -72, FIELD_MAX = 72;
    private final double ZONE_SIZE = 48; // 2 tiles, 48 inches
    private final double SLOWDOWN_FACTOR = 0.4; // Reduce speed to 40% in corners (adjust this factor)

    // Updated zones based on 2 FTC tiles (48 inches)
    private boolean isInCornerZone(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();

        // Top-Left Corner Zone (48" wide)
        boolean inTopLeft     = (x <= FIELD_MIN + ZONE_SIZE && y >= FIELD_MAX - ZONE_SIZE); // Top-left

        // Top-Right Corner Zone (48" wide)
        boolean inTopRight    = (x >= FIELD_MAX - ZONE_SIZE && y >= FIELD_MAX - ZONE_SIZE); // Top-right

        // Bottom-Left Corner Zone (48" wide)
        boolean inBottomLeft  = (x <= FIELD_MIN + ZONE_SIZE && y <= FIELD_MIN + ZONE_SIZE); // Bottom-left

        // Bottom-Right Corner Zone (48" wide)
        boolean inBottomRight = (x >= FIELD_MAX - ZONE_SIZE && y <= FIELD_MIN + ZONE_SIZE); // Bottom-right

        return inTopLeft || inTopRight || inBottomLeft || inBottomRight;
    }

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(-62, 13, 0)); // Set your starting position

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
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();

        // Manual driving
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Default to full speed unless in a corner zone
        double scale = isInCornerZone(pose) ? SLOWDOWN_FACTOR : 1.0;

        // **Adjust this section to set your desired speed when inside the zone**
        // Example: If you want 50% speed in the corner zones, change SLOWDOWN_FACTOR to 0.5

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lf = (y + x + rx) / denominator * scale;  // Left Front motor
        double lr = (y - x + rx) / denominator * scale;  // Left Rear motor
        double rf = (y - x - rx) / denominator * scale;  // Right Front motor
        double rr = (y + x - rx) / denominator * scale;  // Right Rear motor

        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);

        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Slowdown Active", isInCornerZone(pose));  // Check if in slow zone
        telemetry.update();
    }
}
