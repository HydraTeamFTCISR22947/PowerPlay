package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.gamepadHelper;

public class RobotCommands {
    GamepadController robotController;
    gamepadHelper gamepadOneHelper;

    public RobotCommands(HardwareMap hardwareMap, Gamepad gamepad1)
    {
        robotController = new GamepadController(hardwareMap, gamepad1);
        gamepadOneHelper = new gamepadHelper(gamepad1);
    }

    public void drivetrainUpdate()
    {
        robotController.setMotorPowers();

        if(gamepadOneHelper.XOnce())
        {
            robotController.setFieldCentric(!robotController.isFieldCentric());
        }
    }
}
