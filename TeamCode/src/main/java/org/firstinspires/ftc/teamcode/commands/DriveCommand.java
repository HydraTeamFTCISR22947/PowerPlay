package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;
import org.firstinspires.ftc.teamcode.util.RobotCommand;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

public class DriveCommand implements RobotCommand {
    GamepadController robotController;
    boolean redAlliance;

    public DriveCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isRed)
    {
        redAlliance = isRed;
        initCommand(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void runCommand() {
        robotController.update(redAlliance);
    }

    @Override
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        robotController = new GamepadController(hardwareMap, gamepad1, gamepad2, telemetry, redAlliance); // TeleopCommand(gamepad) class functions
    }

}
