package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_VertSlide;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class VertSlide {
    private DcMotor left;
    private DcMotor right;

    private boolean isLeftHolding = false;
    private boolean isRightHolding = false;
    private int leftHoldPosition = 0;
    private int rightHoldPosition = 0;

    public VertSlide(DcMotor L, DcMotor R) {
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

    public void setPower(double rightTrigger, double leftTrigger) {
        double slidePowerInput = rightTrigger - leftTrigger;

        int leftCurrent = left.getCurrentPosition();
        int rightCurrent = right.getCurrentPosition();

        // --- Movement ---
        if (slidePowerInput != 0) {
            isLeftHolding = false;
            isRightHolding = false;

            // Cancel any holding mode
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Soft limit scaling
            double leftScale = 1.0;
            double rightScale = 1.0;
            int slowZone = 200;

            int leftDistToMin = leftCurrent - RC_VertSlide.minPosition;
            int leftDistToMax = RC_VertSlide.maxPosition - leftCurrent;

            int rightDistToMin = rightCurrent - RC_VertSlide.minPosition;
            int rightDistToMax = RC_VertSlide.maxPosition - rightCurrent;

            if (slidePowerInput < 0 && leftDistToMin < slowZone) {
                leftScale = Math.max(0, (double) leftDistToMin / slowZone);
            } else if (slidePowerInput > 0 && leftDistToMax < slowZone) {
                leftScale = Math.max(0, (double) leftDistToMax / slowZone);
            }

            if (slidePowerInput < 0 && rightDistToMin < slowZone) {
                rightScale = Math.max(0, (double) rightDistToMin / slowZone);
            } else if (slidePowerInput > 0 && rightDistToMax < slowZone) {
                rightScale = Math.max(0, (double) rightDistToMax / slowZone);
            }

            double leftPower = Math.max(-1.0, Math.min(1.0, slidePowerInput * leftScale));
            double rightPower = Math.max(-1.0, Math.min(1.0, slidePowerInput * rightScale));

            left.setPower(leftPower);
            right.setPower(rightPower);
        }
        // --- Holding (independent) ---
        else {
            if (!left.isBusy()) {
                if (!isLeftHolding) {
                    leftHoldPosition = leftCurrent;
                    isLeftHolding = true;
                }
                left.setTargetPosition(leftHoldPosition);
                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left.setPower(RC_VertSlide.holdPower);
            }

            if (!right.isBusy()) {
                if (!isRightHolding) {
                    rightHoldPosition = rightCurrent;
                    isRightHolding = true;
                }
                right.setTargetPosition(rightHoldPosition);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setPower(RC_VertSlide.holdPower);
            }
        }

        TelemetryData.slideCount = leftCurrent;
    }

    public void runToMax() {
        left.setTargetPosition(RC_VertSlide.maxPosition);
        right.setTargetPosition(RC_VertSlide.maxPosition);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(1.0);
        right.setPower(1.0);

        isLeftHolding = false;
        isRightHolding = false;

        TelemetryData.slideCount = left.getCurrentPosition();
    }

    public void runToMin() {
        left.setTargetPosition(RC_VertSlide.minPosition);
        right.setTargetPosition(RC_VertSlide.minPosition);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(1.0);
        right.setPower(1.0);

        isLeftHolding = false;
        isRightHolding = false;

        TelemetryData.slideCount = left.getCurrentPosition();
    }

    public void climb() {
        left.setTargetPosition(RC_VertSlide.climbPosition);
        right.setTargetPosition(RC_VertSlide.climbPosition);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(1.0);
        right.setPower(1.0);

        isLeftHolding = false;
        isRightHolding = false;

        TelemetryData.slideCount = left.getCurrentPosition();
    }
}
