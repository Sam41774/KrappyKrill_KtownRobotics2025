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
@Autonomous(name = "RedLeftSideBucket", group = "Robot")
public class RedLeftSideBucket extends LinearOpMode {

    // Define drive motors.
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private ElapsedTime runtime = new ElapsedTime();

    // These constants are based on the motor and wheel setup.
    // Added correction factor to fix overshooting.
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 15.0;       //3 * 5 = 15 for those gearboxes
    static final double WHEEL_DIAMETER_INCHES = 4.094;      // For calculating circumference.
    // Added 2.5% correction to account for consistent overshoot.
    static final double COUNTS_PER_INCH = 32.1094890; //((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI)) * 1.025;
    static final double COUNTS_PER_INCH_STRAFE = 29.06746; // * 1.2307;
    static final double DRIVE_SPEED = 0.3;

    static final double TURN_SPEED = 0.5;

    // PID Controller variables for rotation.
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // PID Coefficients for FAST rotation - aggressive values for speed.
    static final double kP = 0.08;  // Increased proportional gain for faster response
    static final double kI = 0.000; // Minimized integral term to avoid overshooting
    static final double kD = 0.008; // Increased derivative for better damping at speed

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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        Drive driveTrain = new Drive(leftFront, rightFront, leftBack, rightBack, imu);

        imu.resetYaw();

        telemetry.addData("Status", "Encoders Reset");
        telemetry.update();

        waitForStart();

        setStartingPosition(horzSlide, inTakeArm, claw, shoulder, wrist);






        //++++++++++++++++++++++=========================================================================================================================================



        encoderDrive(DRIVE_SPEED, 6, 6,0.0, 5.0, imu);

        encoderStrafe(DRIVE_SPEED,-12,0.0,3.0, imu);

        fastPidRotate(-45, 2.0,imu);





        encoderDrive(DRIVE_SPEED, 16, 16,0.0, 5.0, imu);

        fastPidRotate(25.0,5.0, imu);

        inTakeArm.goDown();

        spinner.takeOut();

        horzSlide.goOut();

        sleep(1000);

        encoderDrive(DRIVE_SPEED,5,5,25.0,2.0,imu);
        encoderDrive(DRIVE_SPEED,-5,-5,25.0,2.0,imu);

        inTakeArm.goUp();

        spinner.stop();

        horzSlide.goIn();

        sleep(1000);

        claw.close();

        fastPidRotate(-30.0, 5.0, imu);

        //sleep(2000);

        vertSlide.runToMax();

        shoulder.outTakePosition();


        encoderStrafe(DRIVE_SPEED, -10.0, 0.0, 5.0, imu);

        sleep(100);

        fastPidRotate(-45.0, 5.0, imu);

        sleep(100);

        encoderDrive(DRIVE_SPEED,-12,-12,-45.0,5.0,imu);

        sleep(100);


        //sleep(2000);

        encoderDrive(DRIVE_SPEED,-5,-5,-45.0, 5.0,imu);

        sleep(100);

        claw.open();

        sleep(5000);








        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);


    }

    private void setStartingPosition(HorzSlide horzSlide, InTakeArm inTakeArm, Claw claw, Shoulder shoulder, Wrist wrist) {
        horzSlide.goIn();
        inTakeArm.goUp();
        claw.open();
        shoulder.inTakePosition();
        wrist.inTake();
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
     *
     * @param targetDegrees Desired angle to turn in degrees (positive = clockwise)
     * @param timeoutS      Maximum time to attempt the turn in seconds
     * @param imu           The IMU sensor for feedback
     */
    public void fastPidRotate(double targetDegrees, double timeoutS, IMU imu) {
        // Reset PID variables.
        lastError = 0;
        integralSum = 0;
        pidTimer.reset();

        double initialAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double targetAngle = initialAngle + Math.toRadians(targetDegrees);

        double minMotorOutput = 0.15; // Minimum power to overcome static friction
        double maxMotorOutput = 1.0;  // Maximum allowable output

        double largeTurnThreshold = Math.toRadians(45); // 45 degrees threshold
        double mediumTurnThreshold = Math.toRadians(10); // 10 degrees threshold

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

            // Integral term with anti-windup.
            integralSum += error * deltaTime;
            double maxIntegralSum = 0.5 / kI;
            if (Math.abs(integralSum) > maxIntegralSum) {
                integralSum = Math.signum(integralSum) * maxIntegralSum;
            }
            double integralTerm = (Math.abs(error) < Math.toRadians(5)) ? (integralSum * kI) : 0;

            // Derivative term with basic filtering.
            double derivative = (error - lastError) / deltaTime;
            derivative = 0.8 * derivative + 0.2 * (error / deltaTime);
            lastError = error;

            double motorOutput = (error * kP) + integralTerm + (derivative * kD);

            // Adaptive power scaling.
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

            // Set motor powers for rotation.
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

        // Apply a brief counter-torque pulse for braking.
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double error = angleWrap(targetAngle - currentAngle);
        double brakePower = -0.2 * Math.signum(error);

        leftFront.setPower(-brakePower);
        leftBack.setPower(-brakePower);
        rightFront.setPower(brakePower);
        rightBack.setPower(brakePower);
        sleep(30);

        // Stop all motors.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addData("Fast PID Rotation", "Complete");
        telemetry.update();
    }

    /**
     * Drives the robot a set distance based on encoder counts with PID control.
     * Modified to include deceleration ramp and braking to prevent overshooting.
     *
     * @param speed            The base motor speed to use.
     * @param leftInches       The distance (in inches) for the left motors.
     * @param rightInches      The distance (in inches) for the right motors.
     * @param targetHeadingDeg The heading in degrees to maintain during the drive (null uses current heading)
     * @param timeoutS         Timeout in seconds for the move.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, Double targetHeadingDeg, double timeoutS, IMU imu) {
        int newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget;

        // PID coefficients for straight driving.
        double driveKp = 0.1;
        double driveKi = 0.0;
        double driveKd = 0.005;

        double lastHeadingError = 0.0;
        double headingIntegralSum = 0.0;
        ElapsedTime pidDriveTimer = new ElapsedTime();

        if (opModeIsActive()) {
            double targetHeading = (targetHeadingDeg != null) ?
                    Math.toRadians(targetHeadingDeg) :
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Use an offset (e.g., 0.75 inch) to reduce commanded distance for momentum overshoot.
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

                // Adjust motor powers based on correction.
                if (correction > 0) {
                    leftPower = basePower;
                    rightPower = basePower - correction;
                } else {
                    leftPower = basePower + correction;
                    rightPower = basePower;
                }
                leftPower = Math.max(0.05, Math.min(1.0, leftPower));
                rightPower = Math.max(0.05, Math.min(1.0, rightPower));

                // Deceleration ramp: if we are within ~3 inches of target, slow down proportionally.
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

            // Apply active braking (brief reverse pulse) to counter momentum.
            double brakePower = 0.8;
            leftFront.setPower(-brakePower);
            rightFront.setPower(-brakePower);
            leftBack.setPower(-brakePower);
            rightBack.setPower(-brakePower);
            sleep(90);

            // Stop all motors.
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
     * Convenience overload that uses current heading if no specific heading is provided.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS, IMU imu) {
        encoderDrive(speed, leftInches, rightInches, null, timeoutS, imu);
    }

    /**
     * Strafes the robot a set distance with enhanced PID control.
     *
     * @param speed            The base motor speed.
     * @param inches           The distance to strafe (positive = right, negative = left).
     * @param targetHeadingDeg The heading in degrees to maintain (null uses current heading).
     * @param timeoutS         Timeout in seconds.
     * @param imu              The IMU sensor.
     */
    /**
     * Strafes the robot a set distance with enhanced PID control and telemetry debugging.
     *
     * @param speed            The base motor speed.
     * @param inches           The distance to strafe (positive = right, negative = left).
     * @param targetHeadingDeg The heading in degrees to maintain (null uses current heading).
     * @param timeoutS         Timeout in seconds.
     * @param imu              The IMU sensor.
     */
    public void encoderStrafe(double speed, double inches, Double targetHeadingDeg, double timeoutS, IMU imu) {
        int newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget;

        // PID coefficients for strafing.
        double strafeKp = 0.5;
        double strafeKi = 0.0002;
        double strafeKd = 0.009;

        double lastHeadingError = 0.0;
        double headingIntegralSum = 0.0;
        ElapsedTime pidStrafeTimer = new ElapsedTime();

        if (opModeIsActive()) {
            // Use either the provided target heading or the current heading.
            double targetHeading = (targetHeadingDeg != null) ?
                    Math.toRadians(targetHeadingDeg) :
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Adjustment to correct for minor bias.
            int adjust = 0;//(inches < 0) ? -1 : (inches > 0 ? 1 : 0);
            double backAdjust = 1; // Extra factor for the back motors.

            // Apply an offset to counter momentum if needed.
            double offsetInches = -1.0;
            double effectiveInches = (inches > 0) ? inches - offsetInches : inches + offsetInches;

            // Calculate target encoder positions using the strafing constant.
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)((effectiveInches + adjust) * COUNTS_PER_INCH_STRAFE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)((effectiveInches + adjust) * COUNTS_PER_INCH_STRAFE);
            newLeftBackTarget  = leftBack.getCurrentPosition() - (int)((effectiveInches + adjust * backAdjust) * COUNTS_PER_INCH_STRAFE);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)((effectiveInches + adjust * backAdjust) * COUNTS_PER_INCH_STRAFE);

            // Set targets for all motors.
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

            // Initial telemetry for debugging.
            telemetry.addData("Strafe Debug", "Starting strafe command");
            telemetry.addData("Effective Inches", effectiveInches);
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.update();

            // PID control loop with debugging telemetry.
            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double headingError = angleWrap(targetHeading - currentHeading);

                double deltaTime = pidStrafeTimer.seconds();
                pidStrafeTimer.reset();
                if (deltaTime < 0.001) deltaTime = 0.001;

                // Update integral sum with anti-windup.
                headingIntegralSum += headingError * deltaTime;
                double maxIntegral = 0.5 / strafeKi;
                if (Math.abs(headingIntegralSum) > maxIntegral) {
                    headingIntegralSum = Math.signum(headingIntegralSum) * maxIntegral;
                }
                double integralTerm = headingIntegralSum * strafeKi;

                // Derivative term.
                double derivative = (headingError - lastHeadingError) / deltaTime;
                lastHeadingError = headingError;

                // Compute the correction term.
                double correction = (headingError * strafeKp) + integralTerm + (derivative * strafeKd);

                // Adjust motor powers based on the computed correction.
                if (inches > 0) {  // Right strafe.
                    if (correction > 0) { // Need to turn right.
                        lfPower = basePower - correction;
                        rfPower = basePower + correction;
                        lbPower = basePower - correction;
                        rbPower = basePower + correction;
                    } else { // Need to turn left.
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

                // Ensure motor power is within acceptable bounds.
                lfPower = Math.max(0.05, Math.min(1.0, lfPower));
                rfPower = Math.max(0.05, Math.min(1.0, rfPower));
                lbPower = Math.max(0.05, Math.min(1.0, lbPower));
                rbPower = Math.max(0.05, Math.min(1.0, rbPower));

                // Set motor powers.
                leftFront.setPower(lfPower);
                rightFront.setPower(rfPower);
                leftBack.setPower(lbPower);
                rightBack.setPower(rbPower);

                // Send detailed telemetry to FTC Dashboard for debugging/tuning.
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

            // Apply brief active braking.
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

            // Stop all motors.
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
