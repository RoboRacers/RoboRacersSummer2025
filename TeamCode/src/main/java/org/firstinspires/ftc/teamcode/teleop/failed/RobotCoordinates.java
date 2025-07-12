package org.firstinspires.ftc.teamcode.teleop.failed;

import android.graphics.Point;

public class RobotCoordinates {
    double xCenterCamera;
    double yCenterCamera;
    double xCoordObj;
    double yCoordObj;
    double distanceFromCamera;
    double cameraHeight;

    public Point getCameraCoordinate() {
        double xCoord = 0;
        double yCoord = Math.sqrt(Math.pow(distanceFromCamera, 2) - Math.pow(cameraHeight, 2)); // Pythagorean theorem

        double xError = xCoordObj - xCenterCamera;
        double pixelToInchRatio = yCoord / yCoordObj;
        xCoord = pixelToInchRatio * xError;

        return new Point((int) Math.round(xCoord), (int) Math.round(yCoord));
    }
}
