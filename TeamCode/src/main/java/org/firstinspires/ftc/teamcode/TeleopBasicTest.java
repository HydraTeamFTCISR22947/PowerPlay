package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Basic teleop drivetrain test", group="Teleop tests")
@Disabled
public class TeleopBasicTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        GamepadController gamepad = new GamepadController(hardwareMap,gamepad1);
        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            gamepad.getMotorsPower();
            gamepad.activateDriveTrainMotors();

            telemetry.addData("Motors: ", "mFR: (%.2f), mFL: (%.2f), mBR: (%.2f), mBL: (%.2f) ", gamepad.get_mFR_power(), gamepad.get_mFL_power(),gamepad.get_mBR_power(),gamepad.get_mBL_power());
            telemetry.update();
        }


    }
}
