package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Thread.sleep;

public class VisionSystem {
    private OpenCvCamera camera;
    private CombinedHSVandAnglePipeline pipeline;
    private PixelToDistanceMapper mapper;

    CombinedHSVandAnglePipeline.TargetColor targetColor;

    public VisionSystem(HardwareMap hardwareMap, Telemetry telemetry, CombinedHSVandAnglePipeline.TargetColor targetColor) {
        this.targetColor = targetColor;
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);




        pipeline = new CombinedHSVandAnglePipeline(targetColor);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1200, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        double[][] calibrationData = {
                {600, 775, 4,  -13, 11},
                {1254,  788, 5, -1.2,  12},
                {1695,  752, 5, 9.6, 20},
                {740, 478, 12, -11.75, 15},
                {905, 509, 10.5, -9, 14},
                {1110, 523, 11, -3, 15},
                {1278, 531, 10.5, 2.2, 20},
                {1486, 522, 12, 10, 25},
                {800, 348, 20, 9.25, 24},
                {1064, 347, 21.5, -1, 24},
                {1307, 394, 20, 8.5, 30}
        };
        mapper = new PixelToDistanceMapper(calibrationData);
    }

    public void setTargetColor(CombinedHSVandAnglePipeline.TargetColor color) {
        pipeline.setTargetColor(color);
    }

    public void triggerSnapshot() throws InterruptedException {
        pipeline.triggerSnapshot();
        sleep(50);
        pipeline.triggerSnapshot();
        sleep(50);
        pipeline.triggerSnapshot();
        sleep(50);
        pipeline.triggerSnapshot();
        sleep(50);
        pipeline.triggerSnapshot();
        sleep(50);
        pipeline.triggerSnapshot();
        sleep(50);
        pipeline.triggerSnapshot();
        sleep(50);
        pipeline.triggerSnapshot();
        sleep(50);
    }

    public boolean hasProcessedSnapshot(){
        return pipeline.hasProcessedSnapshot();
    }

    public PixelToDistanceMapper.DistanceResult getMappedResult() {
        Point center = pipeline.getCenter();
        if(hasProcessedSnapshot())
            return mapper.getDistanceFromPixel(center.x, center.y);
        else{
            return mapper.getDistanceFromPixel(0, 0);
        }
    }

    public double getSampleAngle(){
        return pipeline.getTargetAngle();
    }

    public void stop() {
        camera.stopStreaming();
    }
}
