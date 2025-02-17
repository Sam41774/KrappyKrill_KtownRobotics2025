package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot_Constants.RC_VertSlide;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class VertSlide {
    private DcMotor left;
    private DcMotor right;

    public VertSlide(DcMotor L, DcMotor R){
        left = L;
        right = R;

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double rightTrigger, double leftTrigger){
        double slidePower = rightTrigger-leftTrigger;
        int currentPosition = left.getCurrentPosition();

        if (slidePower > 0 && currentPosition >= RC_VertSlide.maxPosition) {
            // Trying to move up, but we're at or above the max limit.
            left.setPower(0);
            right.setPower(0);

        } else if (slidePower < 0 && currentPosition <= RC_VertSlide.minPosition) {
            // Trying to move down, but we're at or below the min limit.
            left.setPower(0);
            right.setPower(0);

        } else {
            // Within bounds; allow movement.
            left.setPower(slidePower);
            right.setPower(slidePower);

        }
        TelemetryData.slideCount = left.getCurrentPosition();


    }



}
