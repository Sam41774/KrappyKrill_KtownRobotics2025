package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class Shoulder {
    private Servo left;
    private Servo right;

    public Shoulder(Servo l, Servo r){
        this.left = l;
        this.right = r;
    }

    public void inTakePosition(){
        this.left.setPosition(RC_Shoulder.inTakeLeft);
        this.right.setPosition(RC_Shoulder.inTakeRight);
        TelemetryData.shoulderPosition = 0;
    }

    public void outTakePosition(){
        this.left.setPosition(RC_Shoulder.outTakeLeft);
        this.right.setPosition(RC_Shoulder.outTakeRight);
        TelemetryData.shoulderPosition = 1;
    }

    public void clipPosition(){
        this.left.setPosition(RC_Shoulder.clipLeft);
        this.right.setPosition(RC_Shoulder.clipRight);
        TelemetryData.shoulderPosition = 2;
    }
}
