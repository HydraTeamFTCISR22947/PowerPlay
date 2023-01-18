package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous(name = "Auto Right Blue", group = "test")
public class AutoRightBlue extends LinearOpMode {

    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLeftFront = hardwareMap.get(DcMotor.class, "mFL");
        motorLeftBack = hardwareMap.get(DcMotor.class, "mBL");
        motorRightFront = hardwareMap.get(DcMotor.class, "mFR");
        motorRightBack = hardwareMap.get(DcMotor.class, "mBR");

        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // wait till after init
        waitForStart();

        motorLeftBack.setPower(-power);
        motorRightFront.setPower(-power);
        motorLeftFront.setPower(-power);
        motorRightBack.setPower(-power);

        Thread.sleep(750);

        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);

    }


}