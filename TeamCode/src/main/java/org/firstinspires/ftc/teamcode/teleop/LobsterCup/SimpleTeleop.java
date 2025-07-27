package org.firstinspires.ftc.teamcode.teleop.LobsterCup;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teleop.LobsterCup.RobotControl;

@TeleOp(name = "SimpleLobsterTeleOp", group = "Simplified")
public class SimpleTeleop extends OpMode {

    RobotControl robot = new RobotControl();

    boolean lastX = false;
    boolean lastY = false;
    boolean lastA = false;
    boolean lastB = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Intake Specimen (X)
        if (gamepad1.x) {
            robot.pickupSpecimen();
        }

        // Drop Specimen (Y)
        if (gamepad1.y) {
            robot.dropSpecimen();
        }

        // Open Intake Claw (A)
        if (gamepad1.a) {
            robot.openIntakeClaw();
        }

        // Close Intake Claw (B)
        if (gamepad1.b) {
            robot.closeIntakeClaw();
        }

        // Retract intake if limit switch triggered
        robot.retractIntakeSlideIfLimitHit();

        telemetry.addData("Intake Slide Pos", robot.intakeSlide.getCurrentPosition());
        telemetry.addData("Deposit Slide Pos", robot.depositSlide.getCurrentPosition());
        telemetry.update();
    }
}
