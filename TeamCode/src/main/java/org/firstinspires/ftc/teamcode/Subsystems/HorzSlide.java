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

    public void changePosition(double amount){
        double leftPos = this.left.getPosition();
        double rightPos = this.right.getPosition();

        if (amount > 0){
           if (leftPos < RC_HorzSlide.outLeft){
               this.left.setPosition(leftPos + (amount/100));
               this.right.setPosition(rightPos - (amount/100));
           }
           else {
               this.left.setPosition(RC_HorzSlide.outLeft);
               this.right.setPosition(RC_HorzSlide.outRight);
           }
        }
        else if (amount < 0){
            if (leftPos > RC_HorzSlide.inLeft){
                this.left.setPosition(leftPos + (amount/100));
                this.right.setPosition(rightPos - (amount/100));
            }
            else {
                this.left.setPosition(RC_HorzSlide.inLeft);
                this.right.setPosition(RC_HorzSlide.inRight);
            }
        }
    }
}
