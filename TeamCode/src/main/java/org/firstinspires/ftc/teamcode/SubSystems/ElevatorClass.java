package org.firstinspires.ftc.teamcode.SubSystems;

import android.hardware.HardwareBuffer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorClass {

    DcMotor mE;

    HardwareMap hW;

    public static int HIGH;
    public static int MID;
    public static int LOW;
    public static int RETRACT;

    public static int power = 1;
    public enum pos {

        HIGH_ROD,
        MID_ROD,
        LOW_ROD,
        RETRACT

    }

    pos PoseElevator = pos.RETRACT;

   public ElevatorClass(HardwareMap hW)
   {
    this.hW = hW;

    this.mE = hW.get(DcMotor.class, "mE");
    this.mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    

   }

   public void update()
   {
       switch (PoseElevator)
       {
           case HIGH_ROD:

               goToPOS();


       }
   }

   public void goToPOS(int pose)
   {
       mE.setTargetPosition(pose);
       mE.setPower(power);
       mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

   }
}
