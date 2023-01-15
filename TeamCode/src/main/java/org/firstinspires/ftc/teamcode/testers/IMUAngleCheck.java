package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp(group = "Tests")
public class IMUAngleCheck extends LinearOpMode {
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        initIMU();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Heading(RADS):", getRawExternalHeading());
            telemetry.addData("Heading(DEG):", Math.toDegrees(getRawExternalHeading()));

            telemetry.addData("Heading Velocity(RADS):", getExternalHeadingVelocity());
            telemetry.addData("Heading Velocity(DEG):", Math.toDegrees(getExternalHeadingVelocity()));
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

    public double getRawExternalHeading()
    {
        return imu.getAngularOrientation().firstAngle;
    }

    public Double getExternalHeadingVelocity()
    {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }
}