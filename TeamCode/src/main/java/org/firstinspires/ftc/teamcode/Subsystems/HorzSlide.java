package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot_Constants.RC_HorzSlide;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class HorzSlide {
    private Servo left;
    private Servo right;

    public HorzSlide(Servo l, Servo r){
        this.left = l;
        this.right = r;
    }

    public void goIn(){
        this.left.setPosition(RC_HorzSlide.inLeft);
        this.right.setPosition(RC_HorzSlide.inRight);
        TelemetryData.horzSlidePosition = 0;
    }

    public void goOut(){
        this.left.setPosition(RC_HorzSlide.outLeft);
        this.right.setPosition(RC_HorzSlide.outRight);
        TelemetryData.horzSlidePosition = 1;
    }
}
