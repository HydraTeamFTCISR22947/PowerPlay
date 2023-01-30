package org.firstinspires.ftc.teamcode.testers;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="First Transfer Test", group="Tests")
public class TransferTestFeedforward extends LinearOpMode {
    DcMotorEx motor_transfer;
    public static double power = 0.1;

    public static double Kv = 1.1;
    public static double Ka = 0.2;
    public static  double Ks = 0.001;
    public static  double Kg  = 0.1;
    public static  double Kcos = 0;
    public static  double pos = 0;
    public static  double velo = 0;
    public static  double accel = 0;

    @Override
    public void runOpMode()
    {
        this.motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        this.motor_transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            FeedforwardCoefficientsEx coefficientsEx = new FeedforwardCoefficientsEx(Kv,Ka,Ks,
                    Kg,Kcos);
            FeedforwardEx controller = new FeedforwardEx(coefficientsEx);

            motor_transfer.setPower(controller.calculate(pos,velo,accel));

            telemetry.addData("pos: ", motor_transfer.getCurrentPosition());
            telemetry.addData("velo: ", motor_transfer.getVelocity());
            telemetry.update();
        }
    }


}
