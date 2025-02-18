package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drive {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    public Drive(DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR){
        frontLeft = FL;
        frontRight = FR;
        backLeft = BL;
        backRight = BR;

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double y, double x, double rx){
        x *= 1.1;

        y*=.5;
        x*=.5;
        rx*=.5;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }


}
