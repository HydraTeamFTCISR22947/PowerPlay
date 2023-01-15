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
        GamepadHelper gamepadOneHelper = new GamepadHelper(gamepad1);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            robotController.setMotorPowers();

            if(gamepadOneHelper.XOnce())
            {
                robotController.setFieldCentric(!robotController.isFieldCentric());
            }

            telemetry.addData("Motors: ", "mFR: (%.2f), mFL: (%.2f), mBR: (%.2f), mBL: (%.2f) ", robotController.get_mFR_power(), robotController.get_mFL_power(),robotController.get_mBR_power(),robotController.get_mBL_power());
            telemetry.update();
        }


    }
}
