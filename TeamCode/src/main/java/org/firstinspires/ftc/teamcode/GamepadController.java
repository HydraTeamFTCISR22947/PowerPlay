package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class GamepadController
{
    private static final double STRAFE_CORRECTION = 1.1;
    private static final double DRIVETRAIN_MOTOR_POWER = 0.6;

    private Gamepad _currentGamepad = null;

    private double _mFR_power = 0.0;
    private double _mFL_power = 0.0;
    private double _mBR_power = 0.0;
    private double _mBL_power = 0.0;

    private DcMotor _mFR =  null;
    private DcMotor _mFL =  null;
    private DcMotor _mBR =  null;
    private DcMotor _mBL =  null;


    GamepadController(HardwareMap hardwareMap, Gamepad gamepad1)
    {
        this._currentGamepad =gamepad1;

        this._mFR = hardwareMap.get(DcMotor.class, "mFR");
        this._mFL = hardwareMap.get(DcMotor.class,"mFL");
        this._mBR = hardwareMap.get(DcMotor.class, "mBR");
        this._mBL = hardwareMap.get(DcMotor.class, "mBL");

        this.setMotorsDirections();
    }
    void setMotorsDirections()
    {
        this._mFR.setDirection(DcMotor.Direction.FORWARD);
        this._mFL.setDirection(DcMotor.Direction.REVERSE);
        this._mBR.setDirection(DcMotor.Direction.FORWARD);
        this._mBL.setDirection(DcMotor.Direction.REVERSE);
    }

    void getMotorsPower()
    {
        double rightX = 0.0, leftX = 0.0, y = 0.0;

        y = this._currentGamepad.left_stick_y;
        rightX = this._currentGamepad.right_stick_x * STRAFE_CORRECTION;
        leftX = this._currentGamepad.left_stick_x;

        this._mFL_power = Range.clip(y + leftX + rightX, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mBL_power = Range.clip(y - leftX + rightX, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mFR_power = Range.clip(y - leftX - rightX, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mBR_power = Range.clip(y + leftX - rightX, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
    }

    void activateMotors()
    {
        this._mFR.setPower(this._mFR_power);
        this._mFL.setPower(this._mFL_power);
        this._mBR.setPower(this._mBR_power);
        this._mBL.setPower(this._mBL_power);
    }

}
