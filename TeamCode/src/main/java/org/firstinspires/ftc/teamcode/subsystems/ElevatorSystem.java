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

    // encoder values for elevator heights
    public static int BASE_HEIGHT = 0;
    public static int LOW_HEIGHT = 630;
    public static int MID_HEIGHT = 1400;
    public static int ALMOST_MID_HEIGHT = 460;
    public static int HIGH_HEIGHT = 1970;
    public static int RELEASE_OFFSET = 100;
    // power values
    public static double power = 1;
    public static double maxPower = 1;

    // move elevator up/down by this much
    public static int INCREMENT = 50;

    // current target
    int target = 0;
    // should use pid? - https://www.ctrlaltftc.com/the-pid-controller this is pid
    boolean usePID = true;

    // possible states for the elevator to be in
    public enum elevatorState {
        BASE_LEVEL,
        LOW_ROD,
        MID_ROD,
        HIGH_ROD,
    }

    // the current state
    elevatorState liftState = elevatorState.BASE_LEVEL;

    public ElevatorSystem(HardwareMap hardwareMap) {
        // search for elevator and get the motor
        this.mE = hardwareMap.get(DcMotor.class, "mE");

        // reset encoder
        this.mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // if power zero so brake
        this.mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // use first PID
        this.mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // update function that is called in a loop
    public void update(Gamepad gamepad) {
        // check if pid should be used
        if(usePID)
        {
            // if so , switch between the state that the elevator should be in and set target int value
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

            // set target position , set the power and run to that position using first run to position
            mE.setTargetPosition(target);
            mE.setPower(power);
            mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            // dont use first encoder
            mE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // get manual control on elevator and move by the left stick y value of the gamepad
            if(gamepad.left_stick_y != 0 && !gamepad.left_stick_button)
            {
                // range clip makes sure the first parameter value stays between -maxPower and maxPower
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

    /**
     * This function below are autonmous function for the elevator to just go to those positions in one function call
     */
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

    public void almostMidRod()
    {
        mE.setTargetPosition(ALMOST_MID_HEIGHT);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void highRod() {
        mE.setTargetPosition(HIGH_HEIGHT);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void goToPos(int pos) {
        mE.setTargetPosition(pos);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    /**
     * getters and setters
     */
    public static void setBaseHeight(int baseHeight) {
        BASE_HEIGHT = baseHeight;
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


    public static void setLowHeight(int lowHeight) {
        LOW_HEIGHT = lowHeight;
    }

    public int getPosition()
    {
        return mE.getCurrentPosition();
    }

    // this function updates the current height pos (if elevator is in mid height so it will change the value of the mid height
    // to the pos it got as a parameter)
    public void setHeightByPos(int pos)
    {
        switch (liftState)
        {
            case BASE_LEVEL:
                setBaseHeight(pos);
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
