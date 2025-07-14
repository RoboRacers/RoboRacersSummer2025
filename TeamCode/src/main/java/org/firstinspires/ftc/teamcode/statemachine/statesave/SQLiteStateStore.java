// --- File 1: SQLiteStateStore.java ---
package org.firstinspires.ftc.teamcode.statemachine.statesave;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;

import java.util.*;
import java.util.function.DoubleUnaryOperator;

public class SQLiteStateStore {
    private final SQLiteDatabase db;

    public enum RobotStateKey {
        BATTERY_LEVEL,
        MOTOR_POWER,
        POSE_X,
        POSE_Y,
        HEADING,
        MOVEMENT_STATE,
        SLIDE_STATE,
        SLIDE_POSITION
    }

    public enum MovementState {
        MOVING,
        STOPPED,
        JUST_STOPPED
    }

    public enum SlideState {
        EXTENDED,
        EXTENDING,
        RETRACTED,
        RETRACTING
    }

    private final Map<RobotStateKey, StateRule<Double>> ruleMap = new HashMap<>();
    private final Map<RobotStateKey, List<StateSubscriber>> subscriberMap = new HashMap<>();

    public SQLiteStateStore(Context context) {
        RobotStateDBHelper helper = new RobotStateDBHelper(context);
        db = helper.getWritableDatabase();
    }

    public void setRule(RobotStateKey key, StateRule<Double> rule) {
        ruleMap.put(key, rule);
    }

    public void set(RobotStateKey key, double value) {
        StateRule<Double> rule = ruleMap.get(key);
        if (rule != null && !rule.isValid(value)) return;

        ContentValues values = new ContentValues();
        values.put("state_key", key.name());
        values.put("value", String.valueOf(value));
        db.insertWithOnConflict("robot_state", null, values, SQLiteDatabase.CONFLICT_REPLACE);

        notifySubscribers(key, value);
    }

    public void setMovementState(MovementState state) {
        ContentValues values = new ContentValues();
        values.put("state_key", RobotStateKey.MOVEMENT_STATE.name());
        values.put("value", state.name());
        db.insertWithOnConflict("robot_state", null, values, SQLiteDatabase.CONFLICT_REPLACE);
        notifySubscribers(RobotStateKey.MOVEMENT_STATE, state.ordinal());
    }

    public void setSlideState(SlideState state) {
        ContentValues values = new ContentValues();
        values.put("state_key", RobotStateKey.SLIDE_STATE.name());
        values.put("value", state.name());
        db.insertWithOnConflict("robot_state", null, values, SQLiteDatabase.CONFLICT_REPLACE);
        notifySubscribers(RobotStateKey.SLIDE_STATE, state.ordinal());
    }

    public MovementState getMovementState() {
        Cursor cursor = db.rawQuery("SELECT value FROM robot_state WHERE state_key = ?",
                new String[]{RobotStateKey.MOVEMENT_STATE.name()});
        if (cursor.moveToFirst()) {
            String val = cursor.getString(0);
            cursor.close();
            return MovementState.valueOf(val);
        }
        cursor.close();
        return MovementState.STOPPED;
    }

    public SlideState getSlideState() {
        Cursor cursor = db.rawQuery("SELECT value FROM robot_state WHERE state_key = ?",
                new String[]{RobotStateKey.SLIDE_STATE.name()});
        if (cursor.moveToFirst()) {
            String val = cursor.getString(0);
            cursor.close();
            return SlideState.valueOf(val);
        }
        cursor.close();
        return SlideState.RETRACTED;
    }

    public double get(RobotStateKey key) {
        Cursor cursor = db.rawQuery("SELECT value FROM robot_state WHERE state_key = ?", new String[]{key.name()});
        if (cursor.moveToFirst()) {
            String val = cursor.getString(0);
            cursor.close();
            return Double.parseDouble(val);
        }
        cursor.close();
        return 0.0;
    }

    public boolean contains(RobotStateKey key) {
        Cursor cursor = db.rawQuery("SELECT 1 FROM robot_state WHERE state_key = ?", new String[]{key.name()});
        boolean exists = cursor.moveToFirst();
        cursor.close();
        return exists;
    }

    public void update(RobotStateKey key, DoubleUnaryOperator updateFn) {
        double current = get(key);
        double updated = updateFn.applyAsDouble(current);

        StateRule<Double> rule = ruleMap.get(key);
        if (rule != null && !rule.isValid(updated)) return;

        set(key, updated);
    }

    public void subscribe(RobotStateKey key, StateSubscriber subscriber) {
        subscriberMap.computeIfAbsent(key, k -> new ArrayList<>()).add(subscriber);
    }

    private void notifySubscribers(RobotStateKey key, double newValue) {
        List<StateSubscriber> subscribers = subscriberMap.get(key);
        if (subscribers != null) {
            for (StateSubscriber subscriber : subscribers) {
                subscriber.onStateUpdate(key, newValue);
            }
        }
    }

    public void resetDatabase() {
        db.execSQL("DROP TABLE IF EXISTS robot_state");
        db.execSQL("CREATE TABLE robot_state (state_key TEXT PRIMARY KEY, value TEXT)");
    }

    public void close() {
        db.close();
    }
}