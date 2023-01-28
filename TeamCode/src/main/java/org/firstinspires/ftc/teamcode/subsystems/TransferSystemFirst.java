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
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

/*
 * Hardware class for our transfer system using FIRST'S set target position.
 */
@Config
public class TransferSystemFirst
{
    // set transfer levels values
    public static double PICK_UP = 305;
    public static double MID = 160;
    public static double RELEASE = 10;

    public static double TICKS_PER_REV = 751.8;
    public static double TOTAL_DEGREES = 360;      // total engine spin degrees
    public static double GEAR_RATIO = 1;
    public static double power = 0.5;
    double target = 0;
    DcMotor motor_transfer;    // set motor

    // enum for transfer levels
    public enum TransferLevels {
        PICK_UP,
        MID,
        RELEASE
    }

    // set initial transfer level
    public static TransferLevels transferLevel = TransferLevels.RELEASE;

    public TransferSystemFirst(HardwareMap hardwareMap)
    {
        this.motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //
    public void update()
    {
        switch (transferLevel)
        // set levels
        {
            case PICK_UP:
                target = PICK_UP;
                break;
            case MID:
                target = MID;
                break;
            case RELEASE:
                target = RELEASE;
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
        TransferSystemFirst.transferLevel = transferLevel;
    }

}