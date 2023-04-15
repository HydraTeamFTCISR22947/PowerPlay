package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/*
 * Hardware class for our transfer system using FIRST'S set target position.
 */
@Config
public class TransferSystem
{
    // set transfer levels values, expansion means the transfer is at the side of the expansion hub, otherwise control hub
    public static double PICK_UP = 73;
    public static double PICKUP_EXPANSION = 560;
    public static double HIGH = 200;
    public static double HIGH_EXPANSION = 400;
    public static double TERMINAL = 85;
    public static double TERMINAL_EXPANSION = 670;

    public static double TICKS_PER_REV = 751.8;
    public static double TOTAL_DEGREES = 360; // max spin
    public static double GEAR_RATIO = 1;
    double power = 1;
    double maxPower = .25;
    double target = 0;
    DcMotorEx motor_transfer;    // set motor

    boolean usePID = true;

    // enum for transfer levels
    public enum TransferLevels {
        PICK_UP,
        PICK_UP_EXPANSION,
        TERMINAL,
        TERMINAL_EXPANSION,
        HIGH_EXPANSION,
        HIGH
    }

    // set initial transfer level
    public static TransferLevels transferLevel = TransferLevels.PICK_UP;

    public TransferSystem(HardwareMap hardwareMap)
    {
        this.motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset encoder
        this.motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // brake
        this.motor_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // use first pid
        this.motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE); // reverse motor
    }

    // runs in a loop
    public void update(Gamepad gamepad2)  {
        // like the elevator - please read the comments there is the same thing here!
        if(usePID) {

            switch (transferLevel)
            // set levels
            {
                case HIGH_EXPANSION:
                    target = HIGH_EXPANSION;
                    break;
                case PICK_UP_EXPANSION:
                    target = PICKUP_EXPANSION;
                    break;
                case PICK_UP:
                    target = PICK_UP;
                    break;
                case HIGH:
                    target = HIGH;
                    break;
                case TERMINAL:
                    target = TERMINAL;
                    break;
                case TERMINAL_EXPANSION:
                    target = TERMINAL_EXPANSION;
                    break;
            }

            motor_transfer.setTargetPosition(degreesToEncoderTicks(target));
            motor_transfer.setPower(power);
            motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            motor_transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(gamepad2.right_stick_y != 0 && !gamepad2.right_stick_button)
            {
                motor_transfer.setPower(Range.clip(-gamepad2.right_stick_y, -maxPower, maxPower));
            }
            else
            {
                motor_transfer.setPower(0);
            }
        }
    }

    // function to convert degrees to encoder ticks(doesnt work the best)
    public static int degreesToEncoderTicks(double degrees)
    {
        return (int)Math.round((TICKS_PER_REV * GEAR_RATIO) / (TOTAL_DEGREES / degrees));
    }

    public static double encoderTicksToDegrees(int ticks)
    {
        return (TOTAL_DEGREES * ticks) / (TICKS_PER_REV * GEAR_RATIO);
    }

    // getters and setters

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

    public double getTarget() {
        return target;
    }

    /**
     * these functions below are autonmous functions for the transfer to just go to those positions in one function call
     */

    public void pickUpExpansion() {
        motor_transfer.setTargetPosition(degreesToEncoderTicks(PICKUP_EXPANSION));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void pickUp() {
        motor_transfer.setTargetPosition(degreesToEncoderTicks(PICK_UP));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void highPos() {
        motor_transfer.setTargetPosition(degreesToEncoderTicks(HIGH));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void highExpansionPos() {
        motor_transfer.setTargetPosition(degreesToEncoderTicks(HIGH_EXPANSION));
        motor_transfer.setPower(power);
        motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // setter and getter
    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public int currentPos()
    {
        return motor_transfer.getCurrentPosition();
    }

    // this function updates the current transfer pos (if transfer is in high height so it will change the value of the high height
    // to the pos it got as a parameter)
    public void setHeightByPos(int pos)
    {
        switch (transferLevel)
        {
            case TERMINAL:
                TERMINAL = encoderTicksToDegrees(pos);
                break;
            case TERMINAL_EXPANSION:
                TERMINAL_EXPANSION = encoderTicksToDegrees(pos);
                break;
            case HIGH:
                HIGH = encoderTicksToDegrees(pos);
                break;
            case HIGH_EXPANSION:
                HIGH_EXPANSION = encoderTicksToDegrees(pos);
                break;
            case PICK_UP:
                PICK_UP = encoderTicksToDegrees(pos);
                break;
            case PICK_UP_EXPANSION:
                PICKUP_EXPANSION = encoderTicksToDegrees(pos);
                break;
        }
    }
}