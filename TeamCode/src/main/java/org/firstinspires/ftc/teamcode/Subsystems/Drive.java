package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;

public class Drive {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    public Drive(DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, IMU imu) {
        this.frontLeft = FL;
        this.frontRight = FR;
        this.backLeft = BL;
        this.backRight = BR;
        this.imu = imu;

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double y, double x, double rx) {
        x *= 1.1; // Counteract imperfect strafing

        // Get the robot's heading in radians
        double heading = Math.toRadians(getRobotHeading());

        // Rotate joystick input for field-centric movement
        double rotatedX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotatedY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Reduce speeds (adjust scaling if needed)
        rotatedX *= 0.5;
        rotatedY *= 0.5;
        rx *= 0.5;

        // Calculate motor powers
        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1);
        double frontLeftPower = (rotatedY + rotatedX + rx) / denominator;
        double backLeftPower = (rotatedY - rotatedX + rx) / denominator;
        double frontRightPower = (rotatedY - rotatedX - rx) / denominator;
        double backRightPower = (rotatedY + rotatedX - rx) / denominator;

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void turnToAngle(double targetAngle) {
        double Kp = 0.02; // Tune this value for better response
        double error = targetAngle - getRobotHeading();

        while (Math.abs(error) > 2) { // Stop when within ±2 degrees of the target
            error = targetAngle - getRobotHeading();
            double turnPower = Kp * error;

            // Ensure turn power doesn't exceed safe limits
            turnPower = Math.max(-0.5, Math.min(0.5, turnPower));

            // Apply power to rotate the robot
            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backRight.setPower(-turnPower);
        }

        // Stop motors after reaching the target
        stopMotors();
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private double getRobotHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }
}
