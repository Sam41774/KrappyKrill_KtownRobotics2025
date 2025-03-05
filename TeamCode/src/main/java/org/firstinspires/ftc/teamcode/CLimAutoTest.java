package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@Autonomous(name = "Krilly Auto Drive By Encoder", group = "Robot")
public class CLimAutoTest extends LinearOpMode {

    // Define drive motors.
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private ElapsedTime runtime = new ElapsedTime();

    // These constants are based on the motor and wheel setup.
    // Adjust COUNTS_PER_MOTOR_REV if your motors differ.
    static final double COUNTS_PER_MOTOR_REV = 28;    // e.g., TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 15.0;       // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;      // For calculating circumference.
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system hardware.
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // For a typical mecanum or tank drive, the left side may need to be reversed.
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset encoders.
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders.
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Shoulder shoulder = new Shoulder(
                hardwareMap.get(Servo.class, "leftArm"),
                hardwareMap.get(Servo.class, "rightArm")
        );

        Wrist wrist = new Wrist(
                hardwareMap.get(Servo.class, "wrist")
        );

        telemetry.addData("Status", "Encoders Reset");
        telemetry.update();


        MecanumDrive drive = new MecanumDrive(hardwareMap, Pose2d(0,0,0));



        Action myTraj0 = drive.actionBuilder(new Pose2d(0,0,0))
                .splineTo(new Vector2d(5, -9), 90)
                .splineTo(new Vector2d(10, 15), 90)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        TankDrive.FollowTrajectoryAction(myTraj0);
    }
}