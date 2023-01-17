package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class GamepadController
{
    private static double STRAFE_CORRECTION = 1.1;
    private static double DRIVETRAIN_MOTOR_POWER = 0.6;

    public static boolean fieldCentric = false;

    private Gamepad _currentGamepad = null;

    private double _mFR_power = 0.0;
    private double _mFL_power = 0.0;
    private double _mBR_power = 0.0;
    private double _mBL_power = 0.0;

    private DcMotor _mFR =  null;
    private DcMotor _mFL =  null;
    private DcMotor _mBR =  null;
    private DcMotor _mBL =  null;

    BNO055IMU imu;

    public GamepadController(HardwareMap hardwareMap, Gamepad gamepad1)
    {
        this._currentGamepad =gamepad1;

        this._mFR = hardwareMap.get(DcMotor.class, "mFR");
        this._mFL = hardwareMap.get(DcMotor.class,"mFL");
        this._mBR = hardwareMap.get(DcMotor.class, "mBR");
        this._mBL = hardwareMap.get(DcMotor.class, "mBL");

        this.setMotorsDirections();

        initIMU(hardwareMap);
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
        double x = 0.0, y = 0.0, turn = 0.0;

        y = this._currentGamepad.left_stick_y;
        x = this._currentGamepad.right_stick_x * STRAFE_CORRECTION;
        turn = this._currentGamepad.left_stick_x;

        this._mFL_power = Range.clip(y + turn + x, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mBL_power = Range.clip(y - turn + x, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mFR_power = Range.clip(y - turn - x, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mBR_power = Range.clip(y + turn - x, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
    }

    void getMotorsPowerFieldCentric()
    {
        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double x = 0.0, y = 0.0, turn = 0.0;

        y = -this._currentGamepad.left_stick_y;
        x = this._currentGamepad.right_stick_x * STRAFE_CORRECTION;
        turn = this._currentGamepad.left_stick_x;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        this._mFL_power = Range.clip(rotY + rotX + turn, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mBL_power = Range.clip(rotY - rotX + turn, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mFR_power = Range.clip(rotY - rotX - turn, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
        this._mBR_power = Range.clip(rotY + rotX - turn, -DRIVETRAIN_MOTOR_POWER, DRIVETRAIN_MOTOR_POWER);
    }

    void activateDriveTrainMotors()
    {
        this._mFR.setPower(this._mFR_power);
        this._mFL.setPower(this._mFL_power);
        this._mBR.setPower(this._mBR_power);
        this._mBL.setPower(this._mBL_power);
    }

    public void setMotorPowers()
    {
        if(fieldCentric)
        {
            getMotorsPowerFieldCentric();
        }
        else
        {
            getMotorsPower();
        }

        activateDriveTrainMotors();
    }

    void initIMU(HardwareMap hardwareMap)
    {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    public double get_mFR_power()
    {
        return this._mFR_power;
    }
    public double get_mFL_power()
    {
        return this._mFL_power;
    }
    public double get_mBR_power()
    {
        return this._mBR_power;
    }
    public double get_mBL_power()
    {
        return this._mBL_power;
    }

    public static void setFieldCentric(boolean fieldCentric) {
        GamepadController.fieldCentric = fieldCentric;
    }

    public static boolean isFieldCentric() {
        return fieldCentric;
    }
}
