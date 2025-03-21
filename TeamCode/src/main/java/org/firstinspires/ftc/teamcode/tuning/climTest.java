package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
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
import org.firstinspires.ftc.teamcode.TankDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "climylim")
public final class climTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
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



        Pose2d startPose = new Pose2d(72,-9,0);


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineTo(new Vector2d(19, 9), Math.toRadians(90))
                            .splineTo(new Vector2d(54, 54), Math.toRadians(135))
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




        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, startPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineTo(new Vector2d(19, 9), Math.toRadians(0))
                            .splineTo(new Vector2d(54, 54), Math.toRadians(135))
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
            claw.open();*/
        } else {
            throw new RuntimeException();
        }


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
