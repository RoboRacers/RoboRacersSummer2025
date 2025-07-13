package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Persistent TeleOp", group = "Examples")
public class PersistentOpMode extends LinearOpMode {
    private SQLiteStateStore stateStore;

    @Override
    public void runOpMode() {
        stateStore = new SQLiteStateStore(hardwareMap.appContext);

        // Set values
        stateStore.set(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL, 95.0);
        stateStore.set(SQLiteStateStore.RobotStateKey.MOTOR_POWER, 0.7);

        waitForStart();

        while (opModeIsActive()) {
            double battery = stateStore.get(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL);
            telemetry.addData("Battery", battery);

            stateStore.update(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL, level -> level - 0.02);
            telemetry.update();

            sleep(100);
        }

        stateStore.close();
    }
}
