package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Claw;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Spinner;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class Spinner {
    private CRServo left;
    private CRServo right;
    private CRServo top;

    public Spinner(CRServo L, CRServo R, CRServo T){
        this.left = L;
        this.right = R;
        this.top = T;
    }

    public void takeIn(){
        this.left.setPower(RC_Spinner.leftInMax);
        this.right.setPower(RC_Spinner.rightInMax);
        this.top.setPower(RC_Spinner.topInMax);
        TelemetryData.spinnerMode=1;
    }

    public void takeOut(){
        this.left.setPower(RC_Spinner.leftOutMax);
        this.right.setPower(RC_Spinner.rightOutMax);
        this.top.setPower(RC_Spinner.topOutMax);
        TelemetryData.spinnerMode=2;
    }

    public void stop(){
        this.left.setPower(0);
        this.right.setPower(0);
        this.top.setPower(0);
        TelemetryData.spinnerMode=0;
    }

}
