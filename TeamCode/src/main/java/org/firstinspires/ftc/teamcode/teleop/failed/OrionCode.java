//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorImplEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//
////import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp(name = "Orion the Huntsman", group = "0000-Final")
//public class OrionCode extends LinearOpMode {
//
//    Gamepad previousGamepad1 = new Gamepad();
//    Gamepad previousGamepad2 = new Gamepad();
//
//    private FtcDashboard dash = FtcDashboard.getInstance();
////    private List<Action> runningActions = new ArrayList<>();
//
//    DcMotorImplEx intakeMotor;
//    DcMotorImplEx slidesMotor;
//    ServoImplEx extendoLeft;
//    ServoImplEx extendoRight;
//    ServoImplEx depositFlipLeft;
//    ServoImplEx depositFlipRight;
//    ServoImplEx depositV4bServo;
//    ServoImplEx claw;
//
//    ServoImplEx intakeFlipLeft;
//    ServoImplEx intakeFlipRight;
//    ServoImplEx intakeV4b;
//
//    double intakeFlip;
//
//    double depositFlip;
//    double extendo;
//
//    double clawPos;
//    double intake4b;
//    double deposit4b;
//
////    ElapsedTime time = new ElapsedTime();
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
//        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
//
//        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");
//        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
//        depositFlipLeft = hardwareMap.get(ServoImplEx.class, "depositFlipLeft");
//
//        depositFlipRight = hardwareMap.get(ServoImplEx.class, "depositFlipRight");
//        depositV4bServo = hardwareMap.get(ServoImplEx.class, "depositV4bServo");
//        claw = hardwareMap.get(ServoImplEx.class, "claw");
//
//        intakeFlipLeft = hardwareMap.get(ServoImplEx.class, "intakeFlipLeft");
//        intakeFlipRight = hardwareMap.get(ServoImplEx.class, "intakeFlipRight");
//        intakeV4b = hardwareMap.get(ServoImplEx.class, "intakeV4b");
//        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
//        intakeMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
//        extendoLeft.setDirection(Servo.Direction.REVERSE);
//        depositFlipLeft.setDirection(Servo.Direction.REVERSE);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//
//        while (opModeInInit()) {
//            intakeMotor.setPower(0);
//            intake4b = 0.18;
//            intakeFlip = 0.96;
//            intakeFlipLeft.setPosition(intakeFlip-0.01);
//            intakeV4b.setPosition(intake4b);
//            intakeFlipRight.setPosition(intakeFlip);
//        }
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ),
//                    -gamepad1.right_stick_x
//            ));
//
//            drive.updatePoseEstimate();
//
////            TelemetryPacket packet = new TelemetryPacket();
//
//            if (gamepad1.dpad_down) { //intake
//                intake4b = 0.4;
//                intakeFlip = 0.4;
//                intakeFlipLeft.setPosition(intakeFlip-0.01);
//                intakeV4b.setPosition(intake4b);
//                intakeFlipRight.setPosition(intakeFlip);
//
//            } else if (gamepad1.dpad_up) { //neutral
//                intakeMotor.setPower(0);
//                intake4b = 0.4;
//                intakeFlip = 0.65;
//                intakeFlipLeft.setPosition(intakeFlip-0.01);
//                intakeV4b.setPosition(intake4b);
//                intakeFlipRight.setPosition(intakeFlip);
//            }
//
//            if (gamepad1.right_trigger > 0.1 && intakeFlip!= 0.5) {intakeMotor.setPower(1);}
//            else if (gamepad1.left_trigger > 0.1) {intakeMotor.setPower(-1);}
//            else{
//                intakeMotor.setPower(0);
//            }
//
//            if (gamepad1.dpad_right) {
//                slidesMotor.setPower(1);
//            } else if (gamepad1.dpad_left) {
//                slidesMotor.setPower(-1);
//            } else {
//                slidesMotor.setPower(0);
//            }
//
//
//
//            if(gamepad2.dpad_up){ // specimen score
//                depositV4bServo.setPosition(0.25);
//
//                depositFlipLeft.setPosition(0.21);
//                depositFlipRight.setPosition(0.21);
//
//
//                claw.setPosition(0.92);
//
//            }
//            if(gamepad2.dpad_down){ // specimen get into subsystem
//                depositV4bServo.setPosition(0.12);
//                depositFlipLeft.setPosition(0.72);
//                depositFlipRight.setPosition(0.72);
//
//            }
//            if(gamepad2.dpad_right){ // specimen grab
//                depositV4bServo.setPosition(0.5);
//
//                depositFlipLeft.setPosition(0.11);
//                depositFlipRight.setPosition(0.11);
//
//            }
//
//            if (gamepad2.cross){
//                extendoRight.setPosition(gamepad2.left_stick_y );
//                extendoLeft.setPosition(gamepad2.left_stick_y);
//            }
//
//            if (gamepad2.square){
//                depositFlipRight.setPosition(depositFlipRight.getPosition()+0.05);
//                depositFlipLeft.setPosition(depositFlipLeft.getPosition()+0.05);
//            }
//
//            if (gamepad2.circle){
//                depositFlipRight.setPosition(depositFlipRight.getPosition()-0.05);
//                depositFlipLeft.setPosition(depositFlipLeft.getPosition()-0.05);
//            }
//
//
//
//
//
//
//
//
//            if(gamepad2.right_bumper){
////                    clawPos = 0.95;
//                claw.setPosition(0.95);
//
//
//            }else if(gamepad2.left_bumper){
//
//                claw.setPosition(0.7);
//            }
//
//            if(gamepad2.right_trigger>0.1){
////                    clawPos = 0.95;
//                depositV4bServo.setPosition(depositV4bServo.getPosition()+0.05);
//
//
//            }else if(gamepad2.left_trigger>0.1){
//
//                depositV4bServo.setPosition(depositV4bServo.getPosition()-0.05);
//            }
//
//
////                intakeV4b.setPosition(intake4b);
////                intakeFlipLeft.setPosition(intakeFlip-0.01);
////
////                intakeFlipRight.setPosition(intakeFlip);
//
////                depositV4bServo.setPosition(deposit4b);
////
////                depositFlipLeft.setPosition(depositFlip);
////                depositFlipRight.setPosition(depositFlip);
////
////                extendoLeft.setPosition(extendo);
////                extendoRight.setPosition(extendo);
////                claw.setPosition(clawPos);
////                intakeFlipLeft.setPosition(intakeFlip-0.01);
////                intakeV4b.setPosition(intake4b);
////                intakeFlipRight.setPosition(intakeFlip);
//            telemetry.update();
//
//
//        }
//    }
//}