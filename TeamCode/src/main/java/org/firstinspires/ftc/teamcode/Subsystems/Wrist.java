package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Claw;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Wrist;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class Wrist {
    private Servo servo;

    public Wrist(Servo servo){
        this.servo = servo;
    }

    public void inTake(){
        this.servo.setPosition(RC_Wrist.inTake);
        TelemetryData.wristPosition = 1;
    }

    public void clipInTake(){
        this.servo.setPosition(RC_Wrist.clipInTake);
        TelemetryData.wristPosition = 3;
    }

    public void clipOutTake(){
        this.servo.setPosition(RC_Wrist.clipOutTake);
        TelemetryData.wristPosition = 4;
    }

    public void outTake(){
        this.servo.setPosition(RC_Wrist.outTake);
        TelemetryData.wristPosition = 2;
    }

    public void touchBar(){
        this.servo.setPosition(RC_Wrist.bar);
        TelemetryData.wristPosition = 5;
    }

}
