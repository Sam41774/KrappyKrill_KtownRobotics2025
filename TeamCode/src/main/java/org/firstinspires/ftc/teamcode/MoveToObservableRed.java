package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.HorzSlide;
import org.firstinspires.ftc.teamcode.Subsystems.InTakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Spinner;
import org.firstinspires.ftc.teamcode.Subsystems.VertSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@Autonomous(name = "Krilly Auto Drive By Encoder", group = "Robot")
public class MoveToObservableRed extends LinearOpMode {

    // Define drive motors.
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private ElapsedTime runtime = new ElapsedTime();

    // These constants are based on the motor and wheel setup.
    // Adjust COUNTS_PER_MOTOR_REV if your motors differ.
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 15.0;       //3 * 5 = 15 for those lil gear box things
    static final double WHEEL_DIAMETER_INCHES = 4.094;      // For calculating circumference.
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED  = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system hardware.
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

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

        telemetry.addData("Status", "Encoders Reset");
        telemetry.update();

        waitForStart();

        setStartingPosition(horzSlide,inTakeArm,claw,shoulder,wrist);


        // Autonomous driving sequence:
        // 1) Drive forward 48 inches.
        encoderDrive(DRIVE_SPEED,  48, 48, 5.0);

        encoderStrafe(DRIVE_SPEED,-12,3);


        // 2) Turn (for example, a right turn by moving left side forward and right side backward).
        encoderDrive(TURN_SPEED,   -16, 16, 4.0);

        encoderDrive(DRIVE_SPEED,  24, 24, 5.0);

        horzSlide.goOut();


        // 3) Drive backward 24 inches.
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    private void setStartingPosition(HorzSlide horzSlide, InTakeArm inTakeArm, Claw claw, Shoulder shoulder, Wrist wrist){
        horzSlide.goIn();
        inTakeArm.goUp();
        claw.open();
        shoulder.inTakePosition();
        wrist.inTake();
    }

    /**
     * Drives the robot a set distance based on encoder counts.
     *
     * @param speed     The motor speed to use.
     * @param leftInches  The distance (in inches) for the left motors.
     * @param rightInches The distance (in inches) for the right motors.
     * @param timeoutS  Timeout in seconds for the move.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {
            // Calculate new target positions for each motor.
            newLeftFrontTarget  = leftFront.getCurrentPosition()  + (int)(leftInches  * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget   = leftBack.getCurrentPosition()   + (int)(leftInches  * COUNTS_PER_INCH);
            newRightBackTarget  = rightBack.getCurrentPosition()  + (int)(rightInches * COUNTS_PER_INCH);

            // Set target positions.
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Switch to RUN_TO_POSITION mode.
            leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Reset the runtime and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // Loop until the motors reach their target positions or timeout is reached.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                telemetry.addData("Path", "LF: %7d  RF: %7d  LB: %7d  RB: %7d",
                        newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Encoders", "LF: %7d  RF: %7d  LB: %7d  RB: %7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion.
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Switch back to RUN_USING_ENCODER mode.
            leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            //sleep(250);  // Optional pause between moves.
        }
    }
    public void encoderStrafe(double speed, double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        int adjust = 0;

        if (inches < 0){
            adjust -= 1;
        }
        else if (inches > 0){
            adjust += 1;
        }

        double backAdjust = 3;

        // Calculate new target positions for strafing
        newLeftFrontTarget  = leftFront.getCurrentPosition() + (int)((inches + adjust) * COUNTS_PER_INCH);
        newRightFrontTarget = rightFront.getCurrentPosition() - (int)((inches + adjust) * COUNTS_PER_INCH);
        newLeftBackTarget   = leftBack.getCurrentPosition() - (int)((inches + adjust*backAdjust) * COUNTS_PER_INCH);
        newRightBackTarget  = rightBack.getCurrentPosition() + (int)((inches + adjust*backAdjust) * COUNTS_PER_INCH);

        // Set target positions
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        // Set motors to RUN_TO_POSITION
        leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Start movement
        runtime.reset();
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));

        // Loop until finished or timeout
        while ((runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            // Optional telemetry here
        }

        // Stop all motion
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

}
