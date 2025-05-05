package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.HorzSlide;
import org.firstinspires.ftc.teamcode.Subsystems.InTakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Spinner;
import org.firstinspires.ftc.teamcode.Subsystems.VertSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@Config
@Autonomous(name = "BlueLeftSideBucketNew", group = "Robot")
public class BlueLeftSideBucketNew extends LinearOpMode {

    // Define drive motors.
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private ElapsedTime runtime = new ElapsedTime();

    // These constants are based on the motor and wheel setup.
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 15.0;       // 3 * 5 = 15 for those gearboxes
    static final double WHEEL_DIAMETER_INCHES = 4.094;      // For calculating circumference.
    static final double COUNTS_PER_INCH = 32.1094890; // ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI)) * 1.025;
    static final double COUNTS_PER_INCH_STRAFE = 29.06746;
    static final double DRIVE_SPEED = 0.4;

    static final double TURN_SPEED = 0.5;

    // PID Controller variables for rotation.
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // PID Coefficients for FAST rotation.
    static final double kP = 0.08;
    static final double kI = 0.000;
    static final double kD = 0.008;

    // New variable to hold the initial heading (in radians).
    private double startingHeading = 0.0;

    @Override
    public void runOpMode() {

        // Initialize the drive system hardware.
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));

        // Record the starting heading relative to the initial IMU reading.
        imu.resetYaw();
        startingHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        Claw claw = new Claw(hardwareMap.get(Servo.class, "claw"));
        Shoulder shoulder = new Shoulder(hardwareMap.get(Servo.class, "leftArm"), hardwareMap.get(Servo.class, "rightArm"));
        HorzSlide horzSlide = new HorzSlide(hardwareMap.get(Servo.class, "leftHorz"), hardwareMap.get(Servo.class, "rightHorz"));
        InTakeArm inTakeArm = new InTakeArm(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        Spinner spinner = new Spinner(hardwareMap.get(DcMotorEx.class, "spinnerMotor"), hardwareMap.get(CRServo.class, "top"));
        Wrist wrist = new Wrist(hardwareMap.get(Servo.class, "wrist"));
        VertSlide vertSlide = new VertSlide(hardwareMap.get(DcMotorEx.class, "leftVert"), hardwareMap.get(DcMotorEx.class, "rightVert"));
        Drive driveTrain = new Drive(leftFront, rightFront, leftBack, rightBack, imu);

        telemetry.addData("Status", "Encoders Reset & Starting Heading Set");
        telemetry.addData("Starting Heading (deg)", Math.toDegrees(startingHeading));
        telemetry.update();


        waitForStart();

        claw.close();




        // Using the new relative heading concept:
        // For example, a command with heading 0 now means "starting heading."
        sleep(1000);

        setStartingPosition(horzSlide, inTakeArm, claw, shoulder, wrist);

        encoderStrafe(DRIVE_SPEED, -9,0.0,3.0,imu);

        vertSlide.runToMax();

        sleep(3000);

        encoderDrive(DRIVE_SPEED,-4,-4,0.0,2.0,imu);





        shoulder.clipOutTakePosition();

        claw.open();

        sleep(500);

        shoulder.inTakePosition();



        encoderDrive(DRIVE_SPEED, 8,8,0.0,3.0,imu);

        vertSlide.runToMin();

        fastPidRotate(45,2.0,imu);

        //encoderStrafe(DRIVE_SPEED,-2,45.0,2.0,imu);
        shoulder.outTakePosition();

        sleep(500);

        inTakeArm.store();



        encoderDrive(DRIVE_SPEED,10,10,45.0,3.0,imu);
        /*


        //encoderDrive(DRIVE_SPEED, 20,20,45.0,3.0,imu);

        //fastPidRotate();

        sleep(2500);

        inTakeArm.goUp();

        sleep(1500);

        horzSlide.goIn();


        sleep(1500);

        claw.close();

        vertSlide.runToMax();

        spinner.stop();

        encoderDrive(DRIVE_SPEED,-10,-10,45.0,3.0,imu);

        fastPidRotate(0,2.0,imu);

        encoderDrive(DRIVE_SPEED,-6,-6,0.0,2.0,imu);

        shoulder.clipOutTakePosition();

        sleep(3000);

        claw.open();

        sleep(500);

        encoderDrive(DRIVE_SPEED,6,6,0.0,2.0,imu);









        shoulder.outTakePosition();
        wrist.outTake();
        inTakeArm.store();
        vertSlide.runToMin();



         */
        //fastPidRotate(0, 5, imu);











        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(5000);
    }

    private void setStartingPosition(HorzSlide horzSlide, InTakeArm inTakeArm, Claw claw, Shoulder shoulder, Wrist wrist) {
        horzSlide.goIn();
        inTakeArm.goUp();
        claw.close();
        shoulder.clipOutTakePosition();
        wrist.clipOutTake();
    }

    /**
     * Helper to wrap angles between -π and π.
     */
    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    /**
     * Implements fast PID control for robot rotation using the IMU.
     * Now the target angle is computed relative to the initial starting heading.
     *
     * @param targetDegrees Desired relative angle offset from the starting heading (positive = clockwise)
     * @param timeoutS      Maximum time to attempt the turn in seconds.
     * @param imu           The IMU sensor for feedback.
     */
    public void fastPidRotate(double targetDegrees, double timeoutS, IMU imu) {
        lastError = 0;
        integralSum = 0;
        pidTimer.reset();

        // Compute the target heading as the starting heading plus the relative offset.
        double targetAngle = startingHeading + Math.toRadians(targetDegrees);

        double minMotorOutput = 0.15;
        double maxMotorOutput = 1.0;
        double largeTurnThreshold = Math.toRadians(45);
        double mediumTurnThreshold = Math.toRadians(10);

        boolean isComplete = false;
        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive() && !isComplete && loopTimer.seconds() < timeoutS) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error = angleWrap(targetAngle - currentAngle);
            double errorDegrees = Math.toDegrees(error);

            double deltaTime = pidTimer.seconds();
            pidTimer.reset();
            if (deltaTime < 0.001) deltaTime = 0.001;

            integralSum += error * deltaTime;
            double maxIntegralSum = 0.5 / kI;
            if (Math.abs(integralSum) > maxIntegralSum) {
                integralSum = Math.signum(integralSum) * maxIntegralSum;
            }
            double integralTerm = (Math.abs(error) < Math.toRadians(5)) ? (integralSum * kI) : 0;

            double derivative = (error - lastError) / deltaTime;
            derivative = 0.8 * derivative + 0.2 * (error / deltaTime);
            lastError = error;

            double motorOutput = (error * kP) + integralTerm + (derivative * kD);

            if (Math.abs(error) > largeTurnThreshold) {
                motorOutput = Math.signum(motorOutput) * maxMotorOutput;
            } else if (Math.abs(error) > mediumTurnThreshold) {
                double scale = 0.6 + (0.4 * Math.abs(error) / largeTurnThreshold);
                motorOutput = Math.signum(motorOutput) * Math.max(minMotorOutput, Math.abs(motorOutput) * scale);
            }
            if (Math.abs(motorOutput) < minMotorOutput && Math.abs(errorDegrees) > 0.5) {
                motorOutput = Math.signum(motorOutput) * minMotorOutput;
            }
            if (Math.abs(motorOutput) > maxMotorOutput) {
                motorOutput = Math.signum(motorOutput) * maxMotorOutput;
            }
            if (Math.abs(errorDegrees) < 10 && Math.abs(derivative) > 1.0) {
                motorOutput *= 0.8;
            }

            leftFront.setPower(-motorOutput);
            leftBack.setPower(-motorOutput);
            rightFront.setPower(motorOutput);
            rightBack.setPower(motorOutput);

            telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngle));
            telemetry.addData("Current Angle (deg)", Math.toDegrees(currentAngle));
            telemetry.addData("Error (deg)", errorDegrees);
            telemetry.addData("Motor Output", motorOutput);
            telemetry.update();

            if (Math.abs(errorDegrees) < 2.0 && Math.abs(derivative) < 10.0) {
                sleep(50);
                currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                error = angleWrap(targetAngle - currentAngle);
                errorDegrees = Math.toDegrees(error);
                if (Math.abs(errorDegrees) < 2.0) {
                    isComplete = true;
                }
            }
        }

        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double error = angleWrap(targetAngle - currentAngle);
        double brakePower = -0.2 * Math.signum(error);

        leftFront.setPower(-brakePower);
        leftBack.setPower(-brakePower);
        rightFront.setPower(brakePower);
        rightBack.setPower(brakePower);
        sleep(30);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addData("Fast PID Rotation", "Complete");
        telemetry.update();
    }

    /**
     * Drives the robot a set distance with PID control.
     * The target heading is now computed as the starting heading plus any provided offset.
     *
     * @param speed            The base motor speed.
     * @param leftInches       The distance (in inches) for the left motors.
     * @param rightInches      The distance (in inches) for the right motors.
     * @param targetHeadingDeg The desired relative heading (offset in degrees from the starting orientation).
     * @param timeoutS         Timeout in seconds.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, Double targetHeadingDeg, double timeoutS, IMU imu) {
        int newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget;

        double driveKp = 0.2;
        double driveKi = 0.0;
        double driveKd = 0.005;

        double lastHeadingError = 0.0;
        double headingIntegralSum = 0.0;
        ElapsedTime pidDriveTimer = new ElapsedTime();

        if (opModeIsActive()) {
            // Compute target heading relative to the starting heading.
            double targetHeading = (targetHeadingDeg != null) ?
                    startingHeading + Math.toRadians(targetHeadingDeg) :
                    startingHeading;

            double offsetInches = 0;
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)((leftInches - offsetInches) * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)((rightInches - offsetInches) * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int)((leftInches - offsetInches) * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)((rightInches - offsetInches) * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            runtime.reset();
            pidDriveTimer.reset();

            double basePower = Math.abs(speed);
            double leftPower = basePower, rightPower = basePower;

            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double headingError = angleWrap(targetHeading - currentHeading);

                double deltaTime = pidDriveTimer.seconds();
                pidDriveTimer.reset();
                if (deltaTime < 0.001) deltaTime = 0.001;

                headingIntegralSum += headingError * deltaTime;
                double maxIntegral = 0.5 / driveKi;
                if (Math.abs(headingIntegralSum) > maxIntegral) {
                    headingIntegralSum = Math.signum(headingIntegralSum) * maxIntegral;
                }
                double integralTerm = (Math.abs(headingError) < Math.toRadians(5)) ? (headingIntegralSum * driveKi) : 0;

                double derivative = (headingError - lastHeadingError) / deltaTime;
                lastHeadingError = headingError;

                double correction = (headingError * driveKp) + integralTerm + (derivative * driveKd);

                if (correction > 0) {
                    leftPower = basePower;
                    rightPower = basePower - correction;
                } else {
                    leftPower = basePower + correction;
                    rightPower = basePower;
                }
                leftPower = Math.max(0.05, Math.min(1.0, leftPower));
                rightPower = Math.max(0.05, Math.min(1.0, rightPower));

                int leftDistance = Math.abs(newLeftFrontTarget - leftFront.getCurrentPosition());
                int rightDistance = Math.abs(newRightFrontTarget - rightFront.getCurrentPosition());
                int avgDistance = (leftDistance + rightDistance) / 2;
                int decelerationThreshold = (int)(3.0 * COUNTS_PER_INCH);
                if (avgDistance < decelerationThreshold) {
                    double slowDownFactor = Math.max(0.3, avgDistance / (double)decelerationThreshold);
                    leftPower *= slowDownFactor;
                    rightPower *= slowDownFactor;
                }

                leftFront.setPower(leftPower);
                leftBack.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightBack.setPower(rightPower);

                telemetry.addData("Target", "LF: %7d  RF: %7d  LB: %7d  RB: %7d",
                        newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Position", "LF: %7d  RF: %7d  LB: %7d  RB: %7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.addData("Power", "Left: %.2f, Right: %.2f", leftPower, rightPower);
                telemetry.addData("Heading", "Current: %.2f, Target: %.2f, Error: %.2f°",
                        Math.toDegrees(currentHeading), Math.toDegrees(targetHeading), Math.toDegrees(headingError));
                telemetry.update();
            }

            double brakePower = 0.8;
            leftFront.setPower(-brakePower);
            rightFront.setPower(-brakePower);
            leftBack.setPower(-brakePower);
            rightBack.setPower(-brakePower);
            sleep(90);

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

    /**
     * Convenience overload that uses the starting heading if no specific heading is provided.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS, IMU imu) {
        encoderDrive(speed, leftInches, rightInches, null, timeoutS, imu);
    }

    /**
     * Strafes the robot a set distance with enhanced PID control.
     * The target heading is computed relative to the initial starting heading.
     *
     * @param speed            The base motor speed.
     * @param inches           The strafe distance (positive = right, negative = left).
     * @param targetHeadingDeg The desired relative heading (offset in degrees from the starting orientation).
     * @param timeoutS         Timeout in seconds.
     * @param imu              The IMU sensor.
     */
    public void encoderStrafe(double speed, double inches, Double targetHeadingDeg, double timeoutS, IMU imu) {
        int newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget;

        double strafeKp = 0.2;
        double strafeKi = 0.0002;
        double strafeKd = 0.009;

        double lastHeadingError = 0.0;
        double headingIntegralSum = 0.0;
        ElapsedTime pidStrafeTimer = new ElapsedTime();

        if (opModeIsActive()) {
            double targetHeading = (targetHeadingDeg != null) ?
                    startingHeading + Math.toRadians(targetHeadingDeg) :
                    startingHeading;

            int adjust = 0;
            double backAdjust = 1;
            double offsetInches = 0;
            double effectiveInches = (inches > 0) ? inches - offsetInches : inches + offsetInches;

            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)((effectiveInches + adjust) * COUNTS_PER_INCH_STRAFE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)((effectiveInches + adjust) * COUNTS_PER_INCH_STRAFE);
            newLeftBackTarget  = leftBack.getCurrentPosition() - (int)((effectiveInches + adjust * backAdjust) * COUNTS_PER_INCH_STRAFE);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)((effectiveInches + adjust * backAdjust) * COUNTS_PER_INCH_STRAFE);

            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            runtime.reset();
            pidStrafeTimer.reset();

            double basePower = Math.abs(speed);
            double lfPower = basePower, rfPower = basePower, lbPower = basePower, rbPower = basePower;

            telemetry.addData("Strafe Debug", "Starting strafe command");
            telemetry.addData("Effective Inches", effectiveInches);
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.update();

            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double headingError = angleWrap(targetHeading - currentHeading);

                double deltaTime = pidStrafeTimer.seconds();
                pidStrafeTimer.reset();
                if (deltaTime < 0.001) deltaTime = 0.001;

                headingIntegralSum += headingError * deltaTime;
                double maxIntegral = 0.5 / strafeKi;
                if (Math.abs(headingIntegralSum) > maxIntegral) {
                    headingIntegralSum = Math.signum(headingIntegralSum) * maxIntegral;
                }
                double integralTerm = headingIntegralSum * strafeKi;
                double derivative = (headingError - lastHeadingError) / deltaTime;
                lastHeadingError = headingError;
                double correction = (headingError * strafeKp) + integralTerm + (derivative * strafeKd);

                if (inches > 0) {  // Right strafe.
                    if (correction > 0) {
                        lfPower = basePower - correction;
                        rfPower = basePower + correction;
                        lbPower = basePower - correction;
                        rbPower = basePower + correction;
                    } else {
                        lfPower = basePower + Math.abs(correction);
                        rfPower = basePower - Math.abs(correction);
                        lbPower = basePower + Math.abs(correction);
                        rbPower = basePower - Math.abs(correction);
                    }
                } else {  // Left strafe.
                    if (correction > 0) {
                        lfPower = basePower - correction;
                        rfPower = basePower + correction;
                        lbPower = basePower - correction;
                        rbPower = basePower + correction;
                    } else {
                        lfPower = basePower + Math.abs(correction);
                        rfPower = basePower - Math.abs(correction);
                        lbPower = basePower + Math.abs(correction);
                        rbPower = basePower - Math.abs(correction);
                    }
                }

                lfPower = Math.max(0.05, Math.min(1.0, lfPower));
                rfPower = Math.max(0.05, Math.min(1.0, rfPower));
                lbPower = Math.max(0.05, Math.min(1.0, lbPower));
                rbPower = Math.max(0.05, Math.min(1.0, rbPower));

                leftFront.setPower(lfPower);
                rightFront.setPower(rfPower);
                leftBack.setPower(lbPower);
                rightBack.setPower(rbPower);

                telemetry.addData("---- Strafe PID Debug ----", "");
                telemetry.addData("Target Enc (LF)", newLeftFrontTarget);
                telemetry.addData("Target Enc (RF)", newRightFrontTarget);
                telemetry.addData("Target Enc (LB)", newLeftBackTarget);
                telemetry.addData("Target Enc (RB)", newRightBackTarget);
                telemetry.addData("Current Enc (LF)", leftFront.getCurrentPosition());
                telemetry.addData("Current Enc (RF)", rightFront.getCurrentPosition());
                telemetry.addData("Current Enc (LB)", leftBack.getCurrentPosition());
                telemetry.addData("Current Enc (RB)", rightBack.getCurrentPosition());
                telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
                telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
                telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
                telemetry.addData("Delta Time (s)", deltaTime);
                telemetry.addData("Integral Sum", headingIntegralSum);
                telemetry.addData("Derivative", derivative);
                telemetry.addData("Correction", correction);
                telemetry.addData("Motor Power LF", lfPower);
                telemetry.addData("Motor Power RF", rfPower);
                telemetry.addData("Motor Power LB", lbPower);
                telemetry.addData("Motor Power RB", rbPower);
                telemetry.update();
            }

            double brakePower = 0.8;
            if (inches > 0) { // Right strafe braking.
                leftFront.setPower(-brakePower);
                rightFront.setPower(brakePower);
                leftBack.setPower(brakePower);
                rightBack.setPower(-brakePower);
            } else { // Left strafe braking.
                leftFront.setPower(brakePower);
                rightFront.setPower(-brakePower);
                leftBack.setPower(-brakePower);
                rightBack.setPower(brakePower);
            }
            sleep(90);

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Strafe Debug", "Strafe command complete");
            telemetry.update();
        }
    }
}
