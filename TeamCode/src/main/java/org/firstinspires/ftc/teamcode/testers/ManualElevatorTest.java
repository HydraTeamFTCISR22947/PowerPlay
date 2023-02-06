package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@Config
@TeleOp(name="Manual Elevator Test", group="Tests")
public class ManualElevatorTest extends LinearOpMode {
    DcMotor mE;
    public static double power = 0.1;

    @Override
    public void runOpMode()
    {
        this.mE = hardwareMap.get(DcMotor.class, "mE");
        this.mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        GamepadHelper gamepadHelper1 = new GamepadHelper(gamepad1);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            gamepadHelper1.update();
            if(gamepadHelper1.dpadUp())
            {
                mE.setPower(power);
            }
            else if(gamepadHelper1.dpadDown())
            {
                mE.setPower(-power);
            }
            else
            {
                mE.setPower(0);
            }

            telemetry.addData("pos: ", mE.getCurrentPosition());
            telemetry.update();
        }
    }
}
