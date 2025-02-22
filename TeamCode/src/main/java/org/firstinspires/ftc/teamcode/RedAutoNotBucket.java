package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.HorzSlide;
import org.firstinspires.ftc.teamcode.Subsystems.InTakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Spinner;
import org.firstinspires.ftc.teamcode.Subsystems.VertSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;


@Config
@Autonomous(name = "RED_AUTO_NOT_BUCKET", group = "Autonomous")
public class RedAutoNotBucket extends LinearOpMode{

    @Override
    public void runOpMode(){

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

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        tab1,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );




    }
}
