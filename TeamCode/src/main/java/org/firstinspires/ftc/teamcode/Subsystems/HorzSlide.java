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
        if (this.left.getPosition() > RC_HorzSlide.inLeft){
            double Lposition = this.left.getPosition();
            double Rposition = 1-Lposition;
            this.left.setPosition(RC_HorzSlide.inLeft);
            this.right.setPosition(RC_HorzSlide.inRight);
        }
        else if(this.left.getPosition() <= RC_HorzSlide.inLeft) {
            TelemetryData.horzSlidePosition = 0;
        }
    }

    public void goOut(){
        if (this.left.getPosition() < RC_HorzSlide.outLeft){
            double Lposition = this.left.getPosition();
            double Rposition = 1-Lposition;
            this.left.setPosition(RC_HorzSlide.outLeft);
            this.right.setPosition(RC_HorzSlide.outRight);
        }
        else if(this.left.getPosition() <= RC_HorzSlide.outLeft) {
            TelemetryData.horzSlidePosition = 1;
        }
    }
}
