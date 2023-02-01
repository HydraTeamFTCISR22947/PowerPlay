package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Hardware class for our transfer system using FIRST'S set target position.
 */
@Config
public class TransferSystem
{
    // set transfer levels values
    public static double PICK_UP = 40;
    public static double HIGH = 430;

    public static double TICKS_PER_REV = 751.8;
    public static double TOTAL_DEGREES = 360;      // total engine spin degrees
    public static double GEAR_RATIO = 1;
    double power = 1;
    double target = 0;
    DcMotorEx motor_transfer;    // set motor

    // enum for transfer levels
    public enum TransferLevels {
        PICK_UP,
        HIGH
    }

    // set initial transfer level
    public static TransferLevels transferLevel = TransferLevels.PICK_UP;

    public TransferSystem(HardwareMap hardwareMap)
    {
        this.motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //
    public void update()  {
        switch (transferLevel)
        // set levels
        {
            case PICK_UP:
                target = PICK_UP;
                break;
            case HIGH:
                target = HIGH;
                break;
        }

        motor_transfer.setTargetPosition(degreesToEncoderTicks(target));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public DcMotor getMotor()
    {
        return this.motor_transfer;
    }

    public static void setHIGH(double HIGH) {
        TransferSystem.HIGH = HIGH;
    }

    public double getTarget() {
        return target;
    }

    public static double getPickUp() {
        return PICK_UP;
    }

    public static void setPickUp(double pickUp) {
        PICK_UP = pickUp;
    }

    public static double getHIGH() {
        return HIGH;
    }
}