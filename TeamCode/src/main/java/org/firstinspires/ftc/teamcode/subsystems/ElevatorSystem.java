package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSystem
{

    DcMotor mE;

    double BASE_HEIGHT = 0;
    double LOW_HEIGHT = 10;
    double MID_HEIGHT = 25;
    double HIGH_HEIGHT = 35;

    double power;
    double target = 0;

    public static double TICKS_PER_REV = 384.5;
    public static double SPOOL_RADIUS = 2.204725; // in

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
        mE.setTargetPosition(inchesToEncoderTicks(target));
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void baseLevel() {

        mE.setTargetPosition(inchesToEncoderTicks(BASE_HEIGHT));
        if (mE.getCurrentPosition() > inchesToEncoderTicks(BASE_HEIGHT)) {
            mE.setPower(-power);
        }
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
        public void lowRod() {

            mE.setTargetPosition(inchesToEncoderTicks(LOW_HEIGHT));
            if (mE.getCurrentPosition() < inchesToEncoderTicks(LOW_HEIGHT))
            {
                mE.setPower(power);
            }
            mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        public void midRod() {

        mE.setTargetPosition(inchesToEncoderTicks(MID_HEIGHT));
        if (mE.getCurrentPosition() < inchesToEncoderTicks(MID_HEIGHT))
        {
            mE.setPower(power);
        }
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


        public void highRod() {

        mE.setTargetPosition(inchesToEncoderTicks(HIGH_HEIGHT));
        if (mE.getCurrentPosition() < inchesToEncoderTicks(HIGH_HEIGHT))
        {
            mE.setPower(power);
        }
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public static int inchesToEncoderTicks(double inches) {
        return (int)Math.round((inches * TICKS_PER_REV) / (SPOOL_RADIUS * 2 * Math.PI));
    }

    public elevatorState getLiftState() {
        return liftState;
    }

    public void setLiftState(elevatorState liftState) {
        this.liftState = liftState;
    }
}
