package org.firstinspires.ftc.teamcode.statemachine.statesave;

public interface StateSubscriber {
    void onStateUpdate(SQLiteStateStore.RobotStateKey key, double newValue);
}
