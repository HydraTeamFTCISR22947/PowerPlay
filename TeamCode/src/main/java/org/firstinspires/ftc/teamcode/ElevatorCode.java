package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorCode
{

    DcMotor mE;

    int basePosition = 0;
    int lowROD;
    int midROD;
    int highROD;

    double power;

    public enum elevatorState {

        BASE_LEVEL,
        LOW_ROD,
        MID_ROD,
        HIGH_ROD,

    }

    elevatorState liftState;


    public ElevatorCode(HardwareMap hardwareMap)
    {

        this.mE = hardwareMap.get(DcMotor.class, "mE");

        mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mE.setTargetPosition((int) basePosition);

        liftState = elevatorState.BASE_LEVEL;

    }

    public void baseLevel() {

        mE.setTargetPosition(basePosition);
        if (mE.getCurrentPosition() > basePosition) {
            mE.setPower(-power);
        }
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
        public void lowRod() {

            mE.setTargetPosition(lowROD);
            if (mE.getCurrentPosition() < lowROD)
            {
                mE.setPower(power);
            }
            mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        public void midRod() {

        mE.setTargetPosition(lowROD);
        if (mE.getCurrentPosition() < midROD)
        {
            mE.setPower(power);
        }
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


        public void highRod() {

        mE.setTargetPosition(highROD);
        if (mE.getCurrentPosition() < highROD)
        {
            mE.setPower(power);
        }
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

}
