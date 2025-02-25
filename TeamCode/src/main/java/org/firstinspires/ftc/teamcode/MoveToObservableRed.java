package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
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
    static final double COUNTS_PER_MOTOR_REV = 28;    // e.g., TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 15.0;       // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;      // For calculating circumference.
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
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

        Shoulder shoulder = new Shoulder(
                hardwareMap.get(Servo.class,"leftArm"),
                hardwareMap.get(Servo.class, "rightArm")
        );

        Wrist wrist = new Wrist(
                hardwareMap.get(Servo.class, "wrist")
        );

        telemetry.addData("Status", "Encoders Reset");
        telemetry.update();

        waitForStart();

        // Autonomous driving sequence:
        // 1) Drive forward 48 inches.
        encoderDrive(DRIVE_SPEED,  48, 48, 5.0);
        // 2) Turn (for example, a right turn by moving left side forward and right side backward).
        encoderDrive(TURN_SPEED,   -16, 16, 4.0);
        // 3) Drive backward 24 inches.
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);

        shoulder.touchBar();

        wrist.touchBar();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(16000);
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

            sleep(250);  // Optional pause between moves.
        }
    }
}
