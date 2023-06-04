package org.firstinspires.ftc.teamcode.SubSystems;



import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

@Config
public class ElevatorSystem extends SubsystemBase
{
    public enum ElevatorStates
    {
        LOW,
        MID,
        HIGH,
        MANUAL
    };
    public static final double POWER = 1.0;

    public  static final int DPAD_ADDER = 50;
    public static final int HIGH_POS = 600;
    public static final int MID_POS = 300;
    public static final int LOW_POS = 0;

    ElevatorStates state;

     Gamepad gamepad2;

    private DcMotor _mE;
    private boolean _usePid;
    public ElevatorSystem(HardwareMap hardwareMap)
    {
        _usePid = true;

        this._mE = hardwareMap.get(DcMotorEx.class, "mE");
        this._mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this._mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this._mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update()
    {
        switch (state)
        {
            case HIGH:
                this.goToPos(HIGH_POS);
                break;

            case MID:
                this.goToPos(MID_POS);
                break;

            case LOW:
                this.goToPos(LOW_POS);
                break;

            case MANUAL:
                if(gamepad2.dpad_up)
                {
                    this.goToPos(this._mE.getCurrentPosition() + DPAD_ADDER);
                }
                if(gamepad2.dpad_down)
                {
                    this.goToPos(this._mE.getCurrentPosition() - DPAD_ADDER);
                }
                break;
        }
    }

    public void goToPos(int pose)
    {
        this._mE.setTargetPosition(pose);
        this._mE.setPower(POWER);
        this._mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void highRod()
    {
        this.goToPos(HIGH_POS);
    }

    public void midRod()
    {
        this.goToPos(MID_POS);
    }

    public void baseLevel()
    {
        this.goToPos(LOW_POS);
    }
    public int getCurrentPosition()
    {
        return this._mE.getCurrentPosition();
    }

    @Override
    public void periodic() {

        update();
        // This method will be called once per scheduler run
    }
}



