package org.firstinspires.ftc.teamcode.statemachine.statesave;

import android.content.Context;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

public class RobotStateDBHelper extends SQLiteOpenHelper {
    private static final String DB_NAME = "robot_state.db";
    private static final int DB_VERSION = 2;

    // This is a constructor
    public RobotStateDBHelper(Context context) {
        super(context, DB_NAME, null, DB_VERSION);
    }

    @Override
    public void onCreate(SQLiteDatabase db) {
        db.execSQL("CREATE TABLE robot_state (state_key TEXT PRIMARY KEY, value TEXT)");
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        db.execSQL("DROP TABLE IF EXISTS robot_state");
        onCreate(db);
    }
}
