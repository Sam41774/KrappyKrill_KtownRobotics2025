package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_VertSlide;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class VertSlide {
    private DcMotor left;
    private DcMotor right;

    // Store the target encoder position for the slides.
    private int targetPosition;

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

        // Initialize the target position to the current encoder reading.
        targetPosition = left.getCurrentPosition();
    }

    /**
     * Update the vertical slide's position using trigger inputs.
     * When the triggers are pressed, the target encoder position is updated,
     * and the motor mode is set to RUN_TO_POSITION so that the slide actively
     * holds the new position.
     *
     * @param rightTrigger positive input (e.g., raising the slide)
     * @param leftTrigger  positive input (e.g., lowering the slide)
     */
    public void setPower(double rightTrigger, double leftTrigger){
        double slidePowerInput = rightTrigger - leftTrigger;
        int currentPosition = left.getCurrentPosition();

        // If there is input, update the target position based on the scaled trigger value.
        if (slidePowerInput != 0) {
            int positionIncrement = (int)(slidePowerInput * RC_VertSlide.POSITION_SCALE_FACTOR);
            targetPosition += positionIncrement;
        }

        // Clamp the target position to stay within allowed bounds.
        if (targetPosition > RC_VertSlide.maxPosition) {
            targetPosition = RC_VertSlide.maxPosition;
        } else if (targetPosition < RC_VertSlide.minPosition) {
            targetPosition = RC_VertSlide.minPosition;
        }

        // Set the target position for both motors.
        left.setTargetPosition(targetPosition);
        right.setTargetPosition(targetPosition);

        // Switch to RUN_TO_POSITION mode so that the motors will move to (or hold) the target.
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine the power to use.
        // When there is active input, use the magnitude of that input;
        // when holding (no input) use a small constant power.
        double appliedPower = (slidePowerInput != 0) ? Math.abs(slidePowerInput) : RC_VertSlide.holdPower;
        left.setPower(appliedPower);
        right.setPower(appliedPower);

        // Update telemetry.
        TelemetryData.slideCount = left.getCurrentPosition();
    }

    public void runToMax(){
        left.setTargetPosition(RC_VertSlide.maxPosition);
        right.setTargetPosition(RC_VertSlide.maxPosition);

        // Switch to RUN_TO_POSITION mode so that the motors will move to (or hold) the target.
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine the power to use.
        // When there is active input, use the magnitude of that input;
        // when holding (no input) use a small constant power.
        double appliedPower = 1.0;
        left.setPower(appliedPower);
        right.setPower(appliedPower);

        // Update telemetry.
        TelemetryData.slideCount = left.getCurrentPosition();
    }

    public void runToMin(){
        left.setTargetPosition(RC_VertSlide.minPosition);
        right.setTargetPosition(RC_VertSlide.minPosition);

        // Switch to RUN_TO_POSITION mode so that the motors will move to (or hold) the target.
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine the power to use.
        // When there is active input, use the magnitude of that input;
        // when holding (no input) use a small constant power.
        double appliedPower = 1.0;
        left.setPower(appliedPower);
        right.setPower(appliedPower);

        // Update telemetry.
        TelemetryData.slideCount = left.getCurrentPosition();
    }
}
