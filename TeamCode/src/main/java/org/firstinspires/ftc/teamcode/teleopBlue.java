package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CatchAndReleaseCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;

@TeleOp(name = "TeleOp Blue", group = "Main")
public class teleopBlue extends LinearOpMode {
    TeleopCommand teleop;

    @Override
    public void runOpMode() throws InterruptedException {
        teleop = new TeleopCommand(hardwareMap, gamepad1, gamepad2, telemetry, false);

        waitForStart();

        while (opModeIsActive())
        {
            try {
                teleop.runCommand();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}