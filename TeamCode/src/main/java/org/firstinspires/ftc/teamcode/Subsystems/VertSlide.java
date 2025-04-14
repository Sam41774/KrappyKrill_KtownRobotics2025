package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_VertSlide;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class VertSlide {
    private DcMotor left;
    private DcMotor right;

    // Use a single holding flag and hold position since we base control off the right encoder.
    private boolean isHolding = false;
    private int holdPosition = 0;

    public VertSlide(DcMotor L, DcMotor R) {
        left = L;
        right = R;

        // Reverse the left motor so both move the slide in the same physical direction.
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the encoders on both motors
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run using encoders (initially); later modes will be set dynamically.
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply brake behavior on zero power
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double rightTrigger, double leftTrigger) {
        double slidePowerInput = rightTrigger - leftTrigger;
        // Use only the right motor encoder reading for all calculations
        int currentPosition = right.getCurrentPosition();

        // --- Movement: manual control when significant trigger input ---
        if (Math.abs(slidePowerInput) > 0.05) {  // Deadzone threshold
            // Exit holding mode
            isHolding = false;

            // Switch both motors to run without encoder control
            if (left.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (right.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Soft limit scaling (using one "slow zone" for both motors)
            double scale = 1.0;
            int slowZone = 300;  // Expanded zone for gradual deceleration

            if (slidePowerInput < 0) {
                // Moving down - check against minimum bound
                int distToMin = currentPosition - RC_VertSlide.minPosition;
                if (distToMin < slowZone) {
                    scale = Math.max(0.1, (double) distToMin / slowZone);
                }
                // Hard stop margin at minimum
                if (currentPosition <= (RC_VertSlide.minPosition + 5)) {
                    scale = 0;
                }
            } else {
                // Moving up - check against maximum bound
                int distToMax = RC_VertSlide.maxPosition - currentPosition;
                if (distToMax < slowZone) {
                    scale = Math.max(0.1, (double) distToMax / slowZone);
                }
                // Hard stop margin at maximum
                if (currentPosition >= (RC_VertSlide.maxPosition - 5)) {
                    scale = 0;
                }
            }

            double basePowerScale = 1.0;  // You can adjust this if you wish to dampen manual control overall
            double appliedPower = Math.max(-1.0, Math.min(1.0, slidePowerInput * scale * basePowerScale));

            // Apply the computed power to both motors
            left.setPower(appliedPower);
            right.setPower(appliedPower);
        }

        // --- Holding: if there is no manual input, lock the position ---
        else {
            if (!isHolding) {
                // Capture the current position (from the right encoder)
                holdPosition = currentPosition;
                isHolding = true;

                // Switch both motors to position control mode using the hold position
                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                left.setTargetPosition(holdPosition);
                right.setTargetPosition(holdPosition);

                left.setPower(RC_VertSlide.holdPower);
                right.setPower(RC_VertSlide.holdPower);
            }
        }

        // Update telemetry using the right encoder value (which now represents both sides)
        TelemetryData.slideCountR = right.getCurrentPosition();
        TelemetryData.slideCountL = right.getCurrentPosition();
    }

    // Preset positions (run to max, min, clip, and climb) now use the target value across both motors.
    public void runToMax() {
        int targetPosition = RC_VertSlide.maxPosition - 10; // Safety margin
        left.setTargetPosition(targetPosition);
        right.setTargetPosition(targetPosition);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(1.0);
        right.setPower(1.0);

        isHolding = true;

        TelemetryData.slideCountR = right.getCurrentPosition();
        TelemetryData.slideCountL = right.getCurrentPosition();
    }

    public void runToMin() {
        int targetPosition = RC_VertSlide.minPosition + 10; // Safety margin
        left.setTargetPosition(targetPosition);
        right.setTargetPosition(targetPosition);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(1.0);
        right.setPower(1.0);

        isHolding = true;

        TelemetryData.slideCountR = right.getCurrentPosition();
        TelemetryData.slideCountL = right.getCurrentPosition();
    }

    public void clipPosition() {
        int targetPosition = RC_VertSlide.clipPosition;
        left.setTargetPosition(targetPosition);
        right.setTargetPosition(targetPosition);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(1.0);
        right.setPower(1.0);

        isHolding = true;

        TelemetryData.slideCountR = right.getCurrentPosition();
        TelemetryData.slideCountL = right.getCurrentPosition();
    }

    public void climb() {
        int targetPosition = RC_VertSlide.climbPosition;
        left.setTargetPosition(targetPosition);
        right.setTargetPosition(targetPosition);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(1.0);
        right.setPower(1.0);

        isHolding = true;

        TelemetryData.slideCountR = right.getCurrentPosition();
        TelemetryData.slideCountL = right.getCurrentPosition();
    }

    // Modify isBusy() to reflect only the right motor's status
    public boolean isBusy() {
        return right.isBusy();
    }

    // Similarly, atTargetPosition() now checks the error based on the right encoder only.
    public boolean atTargetPosition() {
        int error = Math.abs(right.getTargetPosition() - right.getCurrentPosition());
        int tolerance = 10; // Allowable error range in encoder counts
        return error <= tolerance;
    }
}
