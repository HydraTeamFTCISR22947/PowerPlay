package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorClass {

    DcMotor mE;

    HardwareMap hW;

    public static int HIGH = 1000;
    public static int MID = 800 ;
    public static int LOW = 200 ;
    public static int RETRACT = 0;

    public static int backDown = 50;
    public static int backUp = 70;

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
    this.mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

   }

   public void update()
   {
       switch (PoseElevator)
       {
           case HIGH_ROD:

               this.goToPos(HIGH);
               break;

           case MID_ROD:

               this.goToPos(MID);
               break;

           case LOW_ROD:
               this.goToPos(LOW);
               break;

           default:
              this. goToPos(RETRACT);


       }
   }

   public void goToPos(int pose)
   {
       mE.setTargetPosition(pose);
       mE.setPower(power);
       mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

   }

    public void goToHigh() { goToPos(HIGH);}
    public void goToMid() { goToPos(MID);}
    public void goToLow() { goToPos(LOW);}
    public int getElevatorPose(int currentPose) { return mE.getCurrentPosition(); }

}
