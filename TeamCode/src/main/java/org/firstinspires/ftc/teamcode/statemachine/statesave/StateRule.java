package org.firstinspires.ftc.teamcode.statemachine.statesave;

@FunctionalInterface
public interface StateRule<T> {
    boolean isValid(T value);
}
