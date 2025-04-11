package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot_Constants.RC_inTakeArm;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class InTakeArm {
    private DcMotorEx motor;

    private boolean isHolding = false;
    private int holdPosition = 0;

    public InTakeArm(DcMotorEx m) {
        this.motor = m;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(RC_inTakeArm.direction);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void goUp() {
        motor.setTargetPosition(RC_inTakeArm.minCount);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(RC_inTakeArm.power);
        isHolding = false;

        TelemetryData.inTakeArmCount = motor.getCurrentPosition();
    }

    public void goDown() {
        motor.setTargetPosition(RC_inTakeArm.maxCount);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(RC_inTakeArm.power);
        isHolding = false;

        TelemetryData.inTakeArmCount = motor.getCurrentPosition();
    }

    public void store() {
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(RC_inTakeArm.power);
        isHolding = false;

        TelemetryData.inTakeArmCount = motor.getCurrentPosition();
    }

    public void changePosition(double amount) {
        int currPos = motor.getCurrentPosition();
        int increment = (int)(amount * 10);  // fine-tune this multiplier for sensitivity

        if (increment != 0) {
            int newTarget = currPos + increment;

            // Clamp target within safe range
            newTarget = Math.max(RC_inTakeArm.storePosition, Math.min(RC_inTakeArm.maxCount + 20, newTarget));

            motor.setTargetPosition(newTarget);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(RC_inTakeArm.power);
            isHolding = false;
        } else if (!motor.isBusy()) {
            // --- Auto-hold when idle and no new input ---
            if (!isHolding) {
                holdPosition = currPos;
                isHolding = true;
            }

            motor.setTargetPosition(holdPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(RC_inTakeArm.holdPower);  // Add this to RC_inTakeArm
        }

        TelemetryData.inTakeArmCount = motor.getCurrentPosition();
    }
}
