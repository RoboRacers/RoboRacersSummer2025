package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SLIDES Test", group = "16481")
public class SlidesTest extends LinearOpMode{

        private DcMotor slidesMotor;
        //    private AnalogInput potentiometer;
        private FtcDashboard dashboard;

        double motorPower;
        double maxPower;

        // Configuration variables (tunable via dashboard)
        public static double kP = 0.0;
        public static double kI = 0.00;
        public static double kD = 0.00;
        public static double kF = 0.0;
        public static double targetAngle = 0.0; // Target angle in degrees

        private double integralSum = 0;
        private double lastError = 0;
        private double lastTarget = 0;

        private ElapsedTime timer = new ElapsedTime();

        @Override
        public void runOpMode() throws InterruptedException {
            slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
//        potentiometer = hardwareMap.get(AnalogInput.class, "pot");


            slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // Initialize FTC Dashboard
            dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

            waitForStart();
            timer.reset();

            while (opModeIsActive()) {
                double batteryVoltage = Double.POSITIVE_INFINITY;

                for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
                    double voltage = sensor.getVoltage();
                    if (voltage > 0) {
                        batteryVoltage = Math.min(batteryVoltage, voltage);
                    }
                }

                telemetry.addData("Battery Voltage", "%.2f volts", batteryVoltage);
//
                if (gamepad1.cross){
                    slidesMotor.setPower(1);
                }
                else if (gamepad1.square){
                    slidesMotor.setPower(-0.9);
                }
                else{
                    slidesMotor.setPower(0);
                }
                telemetry.addData("Max Power Used", maxPower);
                telemetry.addData("Slide Power", slidesMotor.getPower());
                telemetry.addData("Slides Pos", slidesMotor.getCurrentPosition());
//                telemetry.addData("Voltaege", );
//            telemetry.addData("Pot Voltage", potentiometer.getVoltage());
                telemetry.update();
                telemetry.update();// Important: Update the dashboard
                dashboard.getTelemetry();
            }
        }

//    private double mapPotentiometerToAngle(double potentiometerValue) {
//
//        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;
//    }

}
