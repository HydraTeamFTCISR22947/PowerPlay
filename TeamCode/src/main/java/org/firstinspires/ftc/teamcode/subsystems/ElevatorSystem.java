package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

@Config
public class ElevatorSystem
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

    public void update(ElevatorStates state, Gamepad gamepad2)
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

    public void midRod()
    {
        this.goToPos(MID_POS);
    }


}
