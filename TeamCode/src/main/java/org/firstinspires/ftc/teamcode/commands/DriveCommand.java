package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;
import org.firstinspires.ftc.teamcode.util.RobotCommand;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

public class DriveCommand implements RobotCommand {
    GamepadController robotController;

    public DriveCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isRed)
    {
        initCommand(hardwareMap, gamepad1, gamepad2, telemetry);
        setRedAlliance(isRed);
    }

    @Override
    public void runCommand() {
        robotController.update();
    }

    @Override
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        robotController = new GamepadController(hardwareMap, gamepad1, gamepad2, telemetry); // TeleopCommand(gamepad) class functions
        robotController.setRedAlliance(true);
    }

    public void setRedAlliance(boolean isRed) {
        robotController.setRedAlliance(isRed);
    }

    public GamepadController getRobotController() {
        return robotController;
    }
}
