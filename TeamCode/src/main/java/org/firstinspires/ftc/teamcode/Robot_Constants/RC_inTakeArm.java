package org.firstinspires.ftc.teamcode.Robot_Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class RC_inTakeArm {

    public static int minCount = 360;
    public static int maxCount = 650;
    public static int storePosition = 10;
    public static double power = 0.8;
    public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;



}
