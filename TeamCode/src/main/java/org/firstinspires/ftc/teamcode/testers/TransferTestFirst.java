package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name="First Transfer Test", group="Tests")
public class TransferTestFirst extends LinearOpMode {
    DcMotorEx motor_transfer;
    public static double power = 0.1;
    public static int pos = 0;

    public static double kP = 0, kI = 0, kD = 0, kF = 0;

    @Override
    public void runOpMode()
    {
        PIDFCoefficients coefficients = new PIDFCoefficients(kP,kI, kD,kF);
        this.motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor_transfer.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefficients);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad1.right_bumper)
            {
                motor_transfer.setTargetPosition(pos);
                motor_transfer.setPower(power);
                motor_transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("pos: ", motor_transfer.getCurrentPosition());
            telemetry.update();
        }
    }


}
