package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_VertSlide;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class VertSlideEncoder {
    private DcMotor left;
    private DcMotor right;

    // For manual control / holding mode.
    private boolean isHolding = false;
    private int holdPosition = 0;

    // Fields used for preset movements.
    private boolean presetActive = false; // Flag for preset movement mode.
    private int presetTarget = 0;         // The target encoder count (from right side) for preset movements.
    private final int slowZone = 300;       // Region near target where power will be scaled down.
    private final double minScale = 0.2;    // Minimum power scale in the slow zone.

    public VertSlideEncoder(DcMotor L, DcMotor R) {
        left = L;
        right = R;

        // Reverse left motor for consistent physical direction.
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        // Since left encoder is broken, only reset the right encoder.
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Left motor runs without encoder; right motor uses its encoder.
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set brake behavior on zero power.
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Manual control update.
     * Call this method during teleop when using trigger input.
     */
    public void setPower(double rightTrigger, double leftTrigger) {
        // If a preset is active, ignore manual power
        if (presetActive) return;

        double slidePowerInput = rightTrigger - leftTrigger;
        int currentPosition = right.getCurrentPosition();

        if (Math.abs(slidePowerInput) > 0.05) {  // Manual movement if input is significant.
            isHolding = false;
            // Ensure the right motor is in manual mode.
            if (right.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            // Left motor remains in manual mode.

            // Determine scale based on proximity to limits.
            double scale = 1.0;
            if (slidePowerInput < 0) {
                // Moving down.
                int distToMin = currentPosition - RC_VertSlide.minPosition;
                if (distToMin < slowZone) {
                    scale = Math.max(0.1, (double) distToMin / slowZone);
                }
                if (currentPosition <= (RC_VertSlide.minPosition + 5)) {
                    scale = 0;
                }
            } else {
                // Moving up.
                int distToMax = RC_VertSlide.maxPosition - currentPosition;
                if (distToMax < slowZone) {
                    scale = Math.max(0.1, (double) distToMax / slowZone);
                }
                if (currentPosition >= (RC_VertSlide.maxPosition - 5)) {
                    scale = 0;
                }
            }

            double appliedPower = slidePowerInput * scale; // Scaling manual power.
            left.setPower(appliedPower);
            right.setPower(appliedPower);
        } else {
            // Enter holding mode when no manual input.
            if (!isHolding) {
                holdPosition = currentPosition;
                isHolding = true;

                // Set right motor to hold position.
                right.setTargetPosition(holdPosition);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setPower(RC_VertSlide.holdPower);
                right.setPower(RC_VertSlide.holdPower);
            }
        }

        // Update telemetry (using right encoder for both sides).
        TelemetryData.slideCountR = right.getCurrentPosition();
        TelemetryData.slideCountL = right.getCurrentPosition();
    }

    // Preset methods. They set the target and enable preset mode.
    public void runToMax() {
        presetTarget = RC_VertSlide.maxPosition - 10;  // Safety margin.
        presetActive = true;
        // Right motor handles position feedback.
        right.setTargetPosition(presetTarget);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Left motor remains open loop.
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToMin() {
        presetTarget = RC_VertSlide.minPosition + 10;  // Safety margin.
        presetActive = true;
        right.setTargetPosition(presetTarget);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void clipPosition() {
        presetTarget = RC_VertSlide.clipPosition;
        presetActive = true;
        right.setTargetPosition(presetTarget);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void climb() {
        presetTarget = RC_VertSlide.climbPosition;
        presetActive = true;
        right.setTargetPosition(presetTarget);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Must be called repeatedly (e.g., in your op mode loop) when a preset is active.
     * This method calculates dynamic power for the left motor based on the error from the right encoder.
     */
    public void updatePreset() {
        if (!presetActive) return;

        int currentPosition = right.getCurrentPosition();
        int error = Math.abs(presetTarget - currentPosition);
        double scale = 1.0;

        if (error < slowZone) {
            // Scale down the motor power as we approach the target.
            scale = Math.max(minScale, (double) error / slowZone);
        }

        // Apply the computed power scale.
        left.setPower(scale);
        right.setPower(scale);

        // Update telemetry.
        TelemetryData.slideCountR = currentPosition;
        TelemetryData.slideCountL = currentPosition;

        // Optionally disable preset mode when within a tolerance (e.g., 10 counts).
        if (error <= 10) {
            presetActive = false;
            // Maintain current position by holding.
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setTargetPosition(currentPosition);
            left.setPower(RC_VertSlide.holdPower);
            right.setPower(RC_VertSlide.holdPower);
        }
    }

    // Returns true if the right motor reports that it is still working to reach its target.
    public boolean isBusy() {
        return right.isBusy();
    }

    // Checks if the right encoder error is within an acceptable tolerance.
    public boolean atTargetPosition() {
        int error = Math.abs(right.getTargetPosition() - right.getCurrentPosition());
        int tolerance = 10; // Acceptable error count.
        return error <= tolerance;
    }
}
