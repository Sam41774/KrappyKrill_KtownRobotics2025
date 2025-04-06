package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Claw;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Spinner;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class Spinner {

    private DcMotorEx motor;
    private CRServo top;

    public Spinner(DcMotorEx M, CRServo T){
        this.motor = M;
        this.top = T;

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void takeIn(){

        this.motor.setPower(RC_Spinner.motorInMax);
        this.top.setPower(RC_Spinner.topInMax);

        TelemetryData.spinnerMode=1;
    }

    public void takeOut(){

        this.motor.setPower(RC_Spinner.motorOutMax);
        this.top.setPower(RC_Spinner.topOutMax);

        TelemetryData.spinnerMode=2;
    }

    public void stop(){

        this.motor.setPower(0);
        this.top.setPower(0);

        TelemetryData.spinnerMode=0;
    }

}
