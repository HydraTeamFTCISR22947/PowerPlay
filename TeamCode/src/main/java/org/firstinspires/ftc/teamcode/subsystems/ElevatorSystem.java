package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

@Config
public class ElevatorSystem {
    DcMotor mE;

    public static int BASE_HEIGHT = 0;
    public static int LOW_HEIGHT = 200;
    public static int MID_HEIGHT = 1200;
    public static int HIGH_HEIGHT = 1850;

    public static double power = 1;
    public static double maxPower = 1;
    public static int INCREMENT = 50;
    int target = 0;
    boolean usePID = true;

    public enum elevatorState {
        BASE_LEVEL,
        LOW_ROD,
        MID_ROD,
        HIGH_ROD,
    }

    elevatorState liftState = elevatorState.BASE_LEVEL;


    public ElevatorSystem(HardwareMap hardwareMap) {
        this.mE = hardwareMap.get(DcMotor.class, "mE");
        this.mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(Gamepad gamepad) {
        if(usePID)
        {
            switch (liftState) {
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

            mE.setTargetPosition(target);
            mE.setPower(power);
            mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            mE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(gamepad.left_stick_y != 0 && !gamepad.left_stick_button)
            {
                mE.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
            }
            else if(gamepad.left_stick_y != 0 && gamepad.left_stick_button)
            {
                mE.setPower(Range.clip(-gamepad.left_stick_y, -1, 1));
            }
            else
            {
                mE.setPower(0);
            }
        }
    }

    public void baseLevel() {
        mE.setTargetPosition(BASE_HEIGHT);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void midRod() {
        mE.setTargetPosition(MID_HEIGHT);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void highRod() {
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

    public static void setMidHeight(int midHeight) {
        MID_HEIGHT = midHeight;
    }

    public static void setHighHeight(int highHeight) {
        HIGH_HEIGHT = highHeight;
    }

    public int currentPos()
    {
        return mE.getCurrentPosition();
    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }


    public static int getLowHeight() {
        return LOW_HEIGHT;
    }

    public static void setLowHeight(int lowHeight) {
        LOW_HEIGHT = lowHeight;
    }

    public static int getMidHeight() {
        return MID_HEIGHT;
    }

    public static int getHighHeight() {
        return HIGH_HEIGHT;
    }

    public int getPosition()
    {
        return mE.getCurrentPosition();
    }

    public void setHeightByPos(int pos)
    {
        switch (liftState)
        {
            case LOW_ROD:
                setLowHeight(pos);
                break;
            case MID_ROD:
                setMidHeight(pos);
                break;
            case HIGH_ROD:
                setHighHeight(pos);
                break;
        }
    }
}
