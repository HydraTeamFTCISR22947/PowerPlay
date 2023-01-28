package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

/*
 * Hardware class for our transfer system using FIRST'S set target position.
 */
@Config
public class TransferSystem
{
    public static double kP = 0.01, kI = 0, kD = 0;

    // set transfer levels values
    public static double PICK_UP = 305;
    public static double MID = 160;
    public static double RELEASE = 10;

    public static double TICKS_PER_REV = 751.8;
    public static double TOTAL_DEGREES = 360;      // total engine spin degrees
    public static double GEAR_RATIO = 1;
    double target = 0;
    DcMotorEx motor_transfer;    // set motor

    double command = 0;
    PIDCoefficients coefficients;
    DoubleSupplier motorPosition;
    BasicPID controller;
    NoFeedforward feedforward;
    RawValue noFilter;
    BasicSystem system;
    // enum for transfer levels
    public enum TransferLevels {
        PICK_UP,
        MID,
        RELEASE
    }

    // set initial transfer level
    public static TransferLevels transferLevel = TransferLevels.RELEASE;

    public TransferSystem(HardwareMap hardwareMap)
    {
        this.motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        motorPosition = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return motor_transfer.getCurrentPosition();
            }
        };

    }

    //
    public void update()
    {
        coefficients = new PIDCoefficients(kP,kI,kD);
        controller = new BasicPID(coefficients);
        feedforward = new NoFeedforward();
        noFilter = new RawValue(motorPosition);
        system = new BasicSystem(noFilter,controller,feedforward);

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

        command = system.update(degreesToEncoderTicks(target));
        motor_transfer.setPower(command);
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