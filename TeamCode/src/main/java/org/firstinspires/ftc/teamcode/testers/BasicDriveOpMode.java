package org.firstinspires.ftc.teamcode.testers;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp
public class BasicDriveOpMode extends OpMode {

    BNO055IMU imu;

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    @Override
    public void init() {
        initIMU();

        /* instantiate motors */
        fL = new Motor(hardwareMap, "mFL");
        fR = new Motor(hardwareMap, "mFR");
        bL = new Motor(hardwareMap, "mBL");
        bR = new Motor(hardwareMap, "mBR");
        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);
    }
    
    @Override
    public void loop() {
        if(gamepad2.right_bumper)
        {
            drive.driveFieldCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX(), imu.getAngularOrientation().firstAngle);
        }
        else if(gamepad2.left_bumper)
        {
            drive.driveFieldCentric(
                    Range.clip(driverOp.getLeftX(), -1, 1),
                    Range.clip(driverOp.getLeftY(), -1, 1),
                    Range.clip(driverOp.getRightX(), -1, 1), imu.getAngularOrientation().firstAngle);
        }
    }

    void initIMU()
    {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);
    }

}