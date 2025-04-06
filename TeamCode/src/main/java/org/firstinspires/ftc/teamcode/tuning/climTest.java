package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.HorzSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Spinner;
import org.firstinspires.ftc.teamcode.Subsystems.VertSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.InTakeArm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous (name = "climylim")
public final class climTest extends LinearOpMode{
    @Override


    public void runOpMode() throws InterruptedException{

        int pos1 = 40;


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
                hardwareMap.get(DcMotorEx.class, "spinnerMotor"),
                hardwareMap.get(CRServo.class,"top")
        );
        Wrist wrist = new Wrist(
                hardwareMap.get(Servo.class, "wrist")
        );
        VertSlide vertSlide = new VertSlide(
                hardwareMap.get(DcMotorEx.class,"leftVert"),
                hardwareMap.get(DcMotorEx.class,"rightVert")
        );



        Pose2d startPose = new Pose2d(-18,-69,0);




        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineTo(new Vector2d(-pos1, -pos1), Math.toRadians(45))
                            .build());


            /*inTakeArm.goDown();
            horzSlide.goOut();
            spinner.takeIn();
            horzSlide.goIn();
            vertSlide.runToMin();
            shoulder.inTakePosition();
            spinner.stop();
            claw.close();
            vertSlide.runToMax();
            shoulder.outTakePosition();
            claw.open(); */

            //Temp comments to stop from using subsytems.




        } else {
            throw new RuntimeException();
        }



        requestOpModeStop();


        /*
            inTakeArm.goDown();
            horzSlide.goOut();
            spinner.takeIn();
            horzSlide.goIn();
            vertSlide.runToMin();
            shoulder.inTakePosition();
            spinner.stop();
            claw.close();
            vertSlide.runToMax();
            shoulder.outTakePosition();
            claw.open();

        */
    }
}
