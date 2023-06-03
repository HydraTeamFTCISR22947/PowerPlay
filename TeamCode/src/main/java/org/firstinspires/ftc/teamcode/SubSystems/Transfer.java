package org.firstinspires.ftc.teamcode.SubSystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Hardware class for our transfer system using FIRST'S set target position.
 */
@Config
public class Transfer
{
    // set transfer levels values
    public static double PICK_UP = 73;
    public static double PICKUP_EXPANSION = 560;
    public static double HIGH = 200;
    public static double HIGH_EXPANSION = 400;
    public static double TERMINAL = 85;
    public static double TERMINAL_EXPANSION = 670;
    // add mid and low levels?

    public static double TICKS_PER_REV = 751.8;
    public static double TOTAL_DEGREES = 360;      // total engine spin degrees
    public static double GEAR_RATIO = 1;
    double power = 1;
    double target = 0;

    DcMotorEx motor_transfer;

    // enum for transfer levels (add more levels)
    public enum TransferLevels {
        PICK_UP,
        PICK_UP_EXPANSION,
        TERMINAL,
        TERMINAL_EXPANSION,
        HIGH_EXPANSION,
        HIGH
    }

    // init stage transfer level
    public static TransferLevels transferLevel = TransferLevels.PICK_UP;

    // Constructor
    public Transfer(HardwareMap hardwareMap)
    {
        this.motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // rotation to ticks
    public static int degreesToEncoderTicks(double degrees)
    {
        return (int)Math.round((TICKS_PER_REV * GEAR_RATIO) / (TOTAL_DEGREES / degrees));
    }

    // ticks to rotation
    public static double encoderTicksToDegrees(int ticks)
    {
        return (TOTAL_DEGREES * ticks) / (TICKS_PER_REV * GEAR_RATIO);
    }

    public int getCurrentPos()
    {
        return motor_transfer.getCurrentPosition();
    }

    public static TransferLevels getTransferLevel() {
        return transferLevel;
    }

    public static void setTransferLevel(TransferLevels transferLevel) {
        Transfer.transferLevel = transferLevel;
    }

    public double getTarget() {
        return target;
    }

    public DcMotor getMotor()
    {
        return this.motor_transfer;
    }

    public void update()  {

        if(transferLevel.equals(HIGH_EXPANSION)){
            target = HIGH_EXPANSION;
        }
        else if(transferLevel.equals(HIGH)){
            target = HIGH;
        }
        else if(transferLevel.equals(PICKUP_EXPANSION)){
            target = PICKUP_EXPANSION;
        }
        else if(transferLevel.equals(PICK_UP)){
            target = PICK_UP;
        }
        else if(transferLevel.equals(TERMINAL_EXPANSION)){
            target = TERMINAL_EXPANSION;
        }
        else if(transferLevel.equals(TERMINAL)){
            target = TERMINAL;
        }

        motor_transfer.setTargetPosition(degreesToEncoderTicks(target));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHeightByPos(int pos)
    {

        if(transferLevel.equals(TERMINAL)){
            TERMINAL = encoderTicksToDegrees(pos);
        }
        else if(transferLevel.equals(TERMINAL_EXPANSION)){
            TERMINAL_EXPANSION = encoderTicksToDegrees(pos);
        }
        else if(transferLevel.equals(HIGH)){
            HIGH = encoderTicksToDegrees(pos);
        }
        else if(transferLevel.equals(HIGH_EXPANSION)){
            HIGH_EXPANSION = encoderTicksToDegrees(pos);
        }
        else if(transferLevel.equals(PICK_UP)){
            PICK_UP = encoderTicksToDegrees(pos);
        }
        else if(transferLevel.equals(PICKUP_EXPANSION)){
            PICKUP_EXPANSION = encoderTicksToDegrees(pos);
        }

    }




    public void toPickupExpansionPos() {
        motor_transfer.setTargetPosition(degreesToEncoderTicks(PICKUP_EXPANSION));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void toPickupPos() {
        motor_transfer.setTargetPosition(degreesToEncoderTicks(PICK_UP));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void toTerminalExpansionPos(){
        motor_transfer.setTargetPosition(degreesToEncoderTicks(TERMINAL_EXPANSION));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void toTerminalPos(){
        motor_transfer.setTargetPosition(degreesToEncoderTicks(TERMINAL));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void toHighPos() {
        motor_transfer.setTargetPosition(degreesToEncoderTicks(HIGH));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void toHighExpansionPos() {
        motor_transfer.setTargetPosition(degreesToEncoderTicks(HIGH_EXPANSION));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> parent of d127fda (k)

    @Override
    public void periodic() {

        update();

    }
>>>>>>> parent of d127fda (k)
}
