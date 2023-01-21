package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.CatchAndReleaseCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.util.RobotCommand;

public class TeleopCommand implements RobotCommand
{
    DriveCommand driveCommand;
    CatchAndReleaseCommand catchAndReleaseCommand;

    public TeleopCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isRed)
    {
        initCommand(hardwareMap, gamepad1, gamepad2, telemetry);
        driveCommand.setRedAlliance(isRed);
    }

    @Override
    public void runCommand()
    {
        driveCommand.runCommand();
        catchAndReleaseCommand.runCommand();
    }

    @Override
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        driveCommand = new DriveCommand(hardwareMap, gamepad1, gamepad2, telemetry, false);
        catchAndReleaseCommand = new CatchAndReleaseCommand(hardwareMap, gamepad1, gamepad2, telemetry);
    }
}