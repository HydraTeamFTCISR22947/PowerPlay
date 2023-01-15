package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/*
 * Hardware class for our transfer system using FIRST'S set target position.
 */
public class TransferSystem
{
    // set transfer levels values
    public static double FINAL = 100;
    public static double MID = 45;
    public static double ZERO = 0;

    public static double TICKS_PER_REV = 751.8 ;
    public static double TOTAL_DEGREES = 360 ;      // total engine spin degrees
    public static double GEAR_RATIO = 1;
    double target = 0;
    double power = 0.2;         // engine power
    DcMotorEx motor;    // set motor

    // enum for transfer levels
    public enum TransferLevels {
        ZERO,
        MID,
        FINAL
    }

    // set initial transfer level
    public static TransferLevels transferLevel = TransferLevels.ZERO;

    public TransferSystem(HardwareMap hardwareMap)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //
    public void update()
    {
        switch (transferLevel)
        // set levels
        {
            case ZERO:
                target = ZERO;
                break;
            case MID:
                target = MID;
                break;
            case FINAL:
                target = FINAL;
                break;
        }

        // set target
        motor.setTargetPosition(degreesToEncoderTicks(target));
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // function to convert degrees to encoder ticks
    public static int degreesToEncoderTicks(double degrees)
    {
        return (int)Math.round((TICKS_PER_REV * GEAR_RATIO) / (TOTAL_DEGREES / degrees));
    }

    public static TransferLevels getTransferLevel() {
        return transferLevel;
    }

    public static void setTransferLevel(TransferLevels transferLevel) {
        TransferSystem.transferLevel = transferLevel;
    }
}