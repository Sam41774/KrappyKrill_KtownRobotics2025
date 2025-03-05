package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class climTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d startPose = new Pose2d(0,0,0);



        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineTo(new Vector2d(15, 15), Math.toRadians(90))
                            .splineTo(new Vector2d(0, 40), Math.toRadians(45))
                            .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, startPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineTo(new Vector2d(15, 15), Math.toRadians(90))
                            .splineTo(new Vector2d(0, 40), Math.toRadians(45))
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
