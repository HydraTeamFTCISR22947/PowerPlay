package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.teleop.CatchAndReleaseCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.ManualFixingCommand;
import org.firstinspires.ftc.teamcode.util.RobotCommand;

public class TeleopCommand implements RobotCommand
{
    DriveCommand driveCommand;
    CatchAndReleaseCommand catchAndReleaseCommand;
    ManualFixingCommand manualFixingCommand;
    boolean redAlliance;

    public TeleopCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isRed)
    {
        redAlliance = isRed;
        initCommand(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void runCommand() throws Exception {
        driveCommand.runCommand();
        catchAndReleaseCommand.runCommand();
        manualFixingCommand.runCommand();
    }

    @Override
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        driveCommand = new DriveCommand(hardwareMap, gamepad1, gamepad2, telemetry, redAlliance);
        catchAndReleaseCommand = new CatchAndReleaseCommand(hardwareMap, gamepad1, gamepad2, telemetry, driveCommand);
        manualFixingCommand = new ManualFixingCommand(hardwareMap, gamepad1, gamepad2, telemetry, catchAndReleaseCommand);
    }
}