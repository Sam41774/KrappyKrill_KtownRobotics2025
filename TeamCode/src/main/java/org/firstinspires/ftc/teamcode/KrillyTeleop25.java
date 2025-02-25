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

import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.Subsystems.HorzSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.InTakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.Spinner;
import org.firstinspires.ftc.teamcode.Subsystems.VertSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



@TeleOp(name = "KRILLYTELLY")
public class KrillyTeleop25 extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException{
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        Gamepad myGamePad = gamepad1;



        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);



        Claw claw = new Claw(
                hardwareMap.get(Servo.class, "claw")
        );
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(Servo.class,"leftArm"),
                hardwareMap.get(Servo.class, "rightArm")
        );
        HorzSlide horzSlide = new HorzSlide(
                hardwareMap.get(Servo.class,"leftHorz"),
                hardwareMap.get(Servo.class, "rightHorz")
        );
        InTakeArm inTakeArm = new InTakeArm(
                hardwareMap.get(DcMotorEx.class, "intakeMotor")
        );
        Spinner spinner = new Spinner(
                hardwareMap.get(CRServo.class, "left"),
                hardwareMap.get(CRServo.class, "right"),
                hardwareMap.get(CRServo.class,"top")
        );
        Wrist wrist = new Wrist(
                hardwareMap.get(Servo.class, "wrist")
        );
        VertSlide vertSlide = new VertSlide(
                hardwareMap.get(DcMotorEx.class,"leftVert"),
                hardwareMap.get(DcMotorEx.class,"rightVert")
        );
        Drive driveTrain = new Drive(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightBack"),
                imu
        );
        DriveRobotCentric driveTrainRobotCentric = new DriveRobotCentric(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightBack")
        );

        driveTrain.resetHeading();

        waitForStart();

        double left_y = gamepad1.left_stick_y;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;
        double left_t = gamepad1.left_trigger;
        double right_t = gamepad1.right_trigger;

        boolean a_state = false;
        boolean b_state = false;
        boolean x_state = false;
        boolean y_state = false;

        boolean rightBumperState = false;
        boolean leftBumperState = false;

        while(opModeIsActive()){

            boolean Apressed = gamepad2.a;
            boolean Bpressed = gamepad2.b;
            boolean Ypressed = gamepad2.y;
            boolean Xpressed = gamepad2.x;

            boolean rbPressed = gamepad2.right_bumper;
            boolean lbPressed = gamepad2.left_bumper;


            left_y = -zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);
            left_t = zeroAnalogInput(gamepad2.left_trigger);
            right_t = zeroAnalogInput(gamepad2.right_trigger);


            if (Apressed && !a_state && TelemetryData.clawPosition == 0) {
                claw.close();
            }else if (Apressed && !a_state && TelemetryData.clawPosition == 1) {
                claw.open();
            }
            a_state = Apressed;

            if (Bpressed && !b_state) {
                shoulder.inTakePosition();
                wrist.inTake();
            }
            b_state = Bpressed;

            if (Xpressed && !x_state) {
                shoulder.clipPosition();
                wrist.clip();
            }
            x_state = Xpressed;

            if (Ypressed != y_state){
                shoulder.outTakePosition();
                wrist.outTake();
            }
            y_state = Ypressed;


            if (gamepad2.dpad_up) {
                horzSlide.goOut();
            } else if (gamepad2.dpad_down) {
                horzSlide.goIn();
            }

            if (gamepad2.dpad_left){
                inTakeArm.goDown();
            } else if (gamepad2.dpad_right){
                inTakeArm.goUp();
            }

            if (rbPressed != rightBumperState && TelemetryData.spinnerMode!=1){
                spinner.takeIn();
            }else if (rbPressed != rightBumperState && TelemetryData.spinnerMode!=0){
                spinner.stop();
            }
            rightBumperState = rbPressed;

            if (lbPressed != leftBumperState && TelemetryData.spinnerMode!=2){
                spinner.takeOut();
            }else if (lbPressed != leftBumperState && TelemetryData.spinnerMode!=0){
                spinner.stop();
            }
            leftBumperState = lbPressed;

            if (right_t - left_t != 0){
                vertSlide.setPower(right_t,left_t);
            }
            else{
                vertSlide.setPower(0,0);
            }

            if(gamepad1.options){
                driveTrain.resetHeading();
            }

            driveTrain.drive(left_y, left_x, right_x);

            telemetry.addData("motor Position" , TelemetryData.inTakeArmCount);
            telemetry.update();
        }



    }

    private double zeroAnalogInput(double input){
        if (Math.abs(input)<0.1){
            input = 0;
        } else if (input < 0) {
            input += .1;
        } else if (input > 0) {
            input -= .1;
        }
        return input;
    }
}


