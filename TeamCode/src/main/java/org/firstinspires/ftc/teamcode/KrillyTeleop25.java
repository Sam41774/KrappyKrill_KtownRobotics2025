package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Claw;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp(name = "KRILLYTELLY")
public class KrillyTeleop25 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        boolean init = true;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));

        Claw claw = new Claw(hardwareMap.get(Servo.class, "claw"));
        Shoulder shoulder = new Shoulder(hardwareMap.get(Servo.class, "leftArm"), hardwareMap.get(Servo.class, "rightArm"));
        HorzSlide horzSlide = new HorzSlide(hardwareMap.get(Servo.class, "leftHorz"), hardwareMap.get(Servo.class, "rightHorz"));
        InTakeArm inTakeArm = new InTakeArm(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        Spinner spinner = new Spinner(hardwareMap.get(DcMotorEx.class, "spinnerMotor"), hardwareMap.get(CRServo.class, "top"));
        Wrist wrist = new Wrist(hardwareMap.get(Servo.class, "wrist"));
        VertSlide vertSlide = new VertSlide(hardwareMap.get(DcMotorEx.class, "leftVert"), hardwareMap.get(DcMotorEx.class, "rightVert"));
        Drive driveTrain = new Drive(hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightBack"),
                imu);
        DriveRobotCentric driveTrainRobotCentric = new DriveRobotCentric(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightBack"));

        driveTrain.resetHeading();
        waitForStart();




        while (opModeIsActive()) {
            if (init) {
                setStartingPosition(horzSlide,inTakeArm,claw,shoulder,wrist);
                init = false;
            }

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double left_y = -zeroAnalogInput(currentGamepad1.left_stick_y);
            double right_y = zeroAnalogInput(currentGamepad1.right_stick_y);
            double left_x = zeroAnalogInput(currentGamepad1.left_stick_x);
            double right_x = zeroAnalogInput(currentGamepad1.right_stick_x);

            double left_t = zeroAnalogInput(currentGamepad2.left_trigger);
            double right_t = zeroAnalogInput(currentGamepad2.right_trigger);
            double left_y2 = -zeroAnalogInput(currentGamepad2.left_stick_y);
            double right_y2 = zeroAnalogInput(currentGamepad2.right_stick_y);



            // Toggle claw
            if (currentGamepad2.a && !previousGamepad2.a) {
                if (TelemetryData.clawPosition == 0) {
                    claw.close();
                } else {
                    claw.open();
                }
            }

            // Wrist + shoulder position switching
            if (currentGamepad2.b && !previousGamepad2.b) {
                shoulder.inTakePosition();
                wrist.inTake();
            }

            if (currentGamepad2.x && !previousGamepad2.x) {
                shoulder.clipPosition();
                wrist.clip();
            }

            if (currentGamepad2.y && !previousGamepad2.y) {
                shoulder.outTakePosition();
                wrist.outTake();
            }

            // Horizontal slide
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                horzSlide.goOut();
            } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                horzSlide.goIn();
            }

            horzSlide.changePosition(left_y2);

            // Intake arm
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                inTakeArm.goDown();
            } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                inTakeArm.goUp();
            }
            else if (right_y2 != 0){
                inTakeArm.changePosition(right_y2);
            }

            // Spinner toggles
            // also for some goddam reason the out and in are reveresed so takeIn() actualy goes out
            if (currentGamepad2.right_bumper) {
                spinner.takeOut();
            }
            else if (currentGamepad2.left_bumper) {
                spinner.takeIn();
            }
            else{
                spinner.stop();
            }

            // Vertical slide
            if (right_t > 0.1 || left_t > 0.1) {
                vertSlide.setPower(right_t,left_t);
            }
            else {
                vertSlide.setPower(0,0);
            }

            if (currentGamepad2.share && !previousGamepad2.share){
                vertSlide.runToMin();
            }
            else if (currentGamepad2.options && !previousGamepad2.options){
                vertSlide.runToMax();
            }


            // reseting heading
            if (currentGamepad1.options && !previousGamepad1.options) {
                driveTrain.resetHeading();
            }


            // the driver has to hit the climb button once in postion
            if (currentGamepad1.a && !previousGamepad1.a) {
                vertSlide.climb();
            }



            // Speed toggle (half speed if holding left bumper)
            double speedMultiplier = currentGamepad1.left_bumper ? RC_Claw.slowDrive : 1.0;

            // Driving with scaled speed
            driveTrain.drive(left_y * speedMultiplier, left_x * speedMultiplier, right_x * speedMultiplier);


            telemetry.addData("motor Position", TelemetryData.inTakeArmCount);
            telemetry.update();
        }
    }

    private double zeroAnalogInput(double input) {
        if (Math.abs(input) < 0.1) return 0;
        return input > 0 ? input - 0.1 : input + 0.1;
    }

    private void setStartingPosition(HorzSlide horzSlide, InTakeArm inTakeArm, Claw claw, Shoulder shoulder, Wrist wrist){
        horzSlide.goIn();
        inTakeArm.goUp();
        claw.open();
        shoulder.inTakePosition();
        wrist.inTake();
    }
}
