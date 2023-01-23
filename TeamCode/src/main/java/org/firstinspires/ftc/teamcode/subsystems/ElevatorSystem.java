package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ElevatorSystem
{

    DcMotor mE;

    public static int BASE_HEIGHT = 0;
    public static int LOW_HEIGHT = 1000;
    public static int MID_HEIGHT = 1700;
    public static int HIGH_HEIGHT = 2470;

    public static double power = 0.2;
    int target = 0;

    public enum elevatorState {
        BASE_LEVEL,
        LOW_ROD,
        MID_ROD,
        HIGH_ROD,
    }

    elevatorState liftState = elevatorState.BASE_LEVEL;


    public ElevatorSystem(HardwareMap hardwareMap)
    {
        this.mE = hardwareMap.get(DcMotor.class, "mE");
        this.mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update()
    {
        switch (liftState)
        {
            case BASE_LEVEL:
                target = BASE_HEIGHT;
                break;
            case LOW_ROD:
                target = LOW_HEIGHT;
                break;
            case MID_ROD:
                target = MID_HEIGHT;
                break;
            case HIGH_ROD:
                target = HIGH_HEIGHT;
                break;
        }

        // set target
        mE.setTargetPosition(target);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void baseLevel()
    {
        mE.setTargetPosition(BASE_HEIGHT);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lowRod()
    {
        mE.setTargetPosition(LOW_HEIGHT);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void midRod()
    {
        mE.setTargetPosition(MID_HEIGHT);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void highRod()
    {
        mE.setTargetPosition(HIGH_HEIGHT);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public elevatorState getLiftState() {
        return liftState;
    }

    public void setLiftState(elevatorState liftState) {
        this.liftState = liftState;
    }
}
