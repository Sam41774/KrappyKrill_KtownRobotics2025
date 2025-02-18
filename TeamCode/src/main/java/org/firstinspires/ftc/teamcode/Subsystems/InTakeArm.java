package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot_Constants.RC_inTakeArm;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class InTakeArm {
    private DcMotorEx motor;


    public InTakeArm(DcMotorEx m){
        this.motor = m;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(RC_inTakeArm.direction);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void goUp(){
        if (this.motor.getCurrentPosition() > RC_inTakeArm.minCount){
            int position = this.motor.getCurrentPosition();
            this.motor.setTargetPosition(RC_inTakeArm.minCount);
            this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.motor.setPower(RC_inTakeArm.power);
        }
        else if(this.motor.getCurrentPosition() <= RC_inTakeArm.minCount) {
            TelemetryData.inTakeArmPosition = 1;

        }

        TelemetryData.inTakeArmCount = this.motor.getCurrentPosition();
    }

    public void goDown(){
        if (this.motor.getCurrentPosition() < RC_inTakeArm.maxCount){
            int position = this.motor.getCurrentPosition();
            this.motor.setTargetPosition(RC_inTakeArm.maxCount);
            this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.motor.setPower(RC_inTakeArm.power);
        }
        else if(this.motor.getCurrentPosition() >= RC_inTakeArm.maxCount) {
            TelemetryData.inTakeArmPosition = 2;
        }

        TelemetryData.inTakeArmCount = this.motor.getCurrentPosition();
    }
}
