package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.GamepadController;
import org.firstinspires.ftc.teamcode.util.RobotCommand;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

public class DriveCommand implements RobotCommand {
    GamepadController robotController;
    GamepadHelper gamepadOneHelper;

    public DriveCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2)
    {
        initCommand(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void runCommand() {
        robotController.setMotorPowers();

        if(gamepadOneHelper.XOnce())
        {
            robotController.setFieldCentric(!robotController.isFieldCentric());
        }
    }

    @Override
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        robotController = new GamepadController(hardwareMap, gamepad1);
        gamepadOneHelper = new GamepadHelper(gamepad1);
    }


}
