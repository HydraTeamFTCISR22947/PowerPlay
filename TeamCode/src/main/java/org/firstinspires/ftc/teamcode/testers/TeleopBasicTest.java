package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GamepadController;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name="Drivetrain Test", group="Tests")
public class TeleopBasicTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        GamepadController robotController = new GamepadController(hardwareMap,gamepad1);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad2.right_bumper)
            {
                robotController.setDrivetrainMotorPower(.9);
            }
            else if(gamepad2.left_bumper)
            {
                robotController.setDrivetrainMotorPower(.2);
            }
            else
            {
                robotController.setDrivetrainMotorPower(.6);
            }

            robotController.setMotorPowers();

            telemetry.addData("Motors: ", "mFR: (%.2f), mFL: (%.2f), mBR: (%.2f), mBL: (%.2f) ", robotController.get_mFR_power(), robotController.get_mFL_power(),robotController.get_mBR_power(),robotController.get_mBL_power());
            telemetry.update();
        }


    }
}
