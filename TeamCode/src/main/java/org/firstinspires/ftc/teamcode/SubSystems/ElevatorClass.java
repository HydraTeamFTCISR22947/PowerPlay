package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorClass {

    DcMotor mE;

    HardwareMap hW;

    public static int HIGH;
    public static int MID;
    public static int LOW;
    public static int RETRACT;
    public enum pos {

        HIGH_ROD,
        MID_ROD,
        LOW_ROD,
        RETRACT

    }

    pos PoseElevator = pos.RETRACT;

   public ElevatorClass()
   {


   }

}
