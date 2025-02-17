package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot_Constants.RC_Claw;
import org.firstinspires.ftc.teamcode.Robot_Constants.TelemetryData;

public class Claw {
    private Servo clawServo;

    public Claw(Servo servo){
        this.clawServo = servo;
    }

    public void open(){
        this.clawServo.setPosition(RC_Claw.open);
        TelemetryData.clawPosition = 0;
    }

    public void close(){
        this.clawServo.setPosition(RC_Claw.close);
        TelemetryData.clawPosition = 1;
    }

}
