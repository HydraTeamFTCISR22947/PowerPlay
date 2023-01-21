package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CatchAndReleaseCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;

@TeleOp(name = "TeleOp Red", group = "Main")
public class teleopRed extends LinearOpMode {
    TeleopCommand teleop;

    @Override
    public void runOpMode() throws InterruptedException {
        teleop = new TeleopCommand(hardwareMap, gamepad1, gamepad2, telemetry, true);

        waitForStart();

        while (opModeIsActive())
        {
            teleop.runCommand();
        }
    }
}