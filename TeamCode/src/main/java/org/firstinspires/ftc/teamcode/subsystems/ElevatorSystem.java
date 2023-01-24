package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

@Config
public class ElevatorSystem
{
    public static double kP = 0, kI = 0, kD = 0;
    PIDCoefficients coefficients = new PIDCoefficients(kP,kI,kD);
    DoubleSupplier motorPosition;
    BasicPID controller = new BasicPID(coefficients);
    NoFeedforward feedforward = new NoFeedforward();
    RawValue noFilter = new RawValue(motorPosition);
    BasicSystem system = new BasicSystem(noFilter,controller,feedforward);

    DcMotor mE;

    public static int BASE_HEIGHT = 0;
    public static int LOW_HEIGHT = 1000;
    public static int MID_HEIGHT = 1700;
    public static int HIGH_HEIGHT = 2470;

    public static double power = 0.2;
    int target = 0;
    double command = 0;

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
        this.mE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorPosition = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return mE.getCurrentPosition();
        }
    };
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

        command = system.update(target);
        //mE.setPower(Range.clip(command, -power, power));
        mE.setPower(command);
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
