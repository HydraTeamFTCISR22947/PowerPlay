package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@Config
@TeleOp(name="Manual Transfer Test", group="Tests")
public class TransferTestManual extends LinearOpMode {
    DcMotor motor_transfer;
    public static double power = 0.1;

    @Override
    public void runOpMode()
    {
        this.motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        GamepadHelper gamepadHelper1 = new GamepadHelper(gamepad1);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            gamepadHelper1.update();
            if(gamepadHelper1.dpadUp())
            {
                motor_transfer.setPower(power);
            }
            else if(gamepadHelper1.dpadDown())
            {
                motor_transfer.setPower(-power);
            }
            else
            {
                motor_transfer.setPower(0);
            }

            telemetry.addData("pos: ", motor_transfer.getCurrentPosition());
            telemetry.update();
        }
    }
}
