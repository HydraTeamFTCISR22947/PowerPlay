package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.teleop.CatchAndReleaseCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.ManualFixingCommand;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;

@TeleOp(name = "TeleOp Red", group = "Main")
public class teleopRed extends LinearOpMode {
    CatchAndReleaseCommand catchAndReleaseCommand;
    ManualFixingCommand manualFixingCommand;
    GamepadController robotController;

    @Override
    public void runOpMode() throws InterruptedException {
        catchAndReleaseCommand = new CatchAndReleaseCommand(hardwareMap, gamepad1, gamepad2, telemetry);
        manualFixingCommand = new ManualFixingCommand(hardwareMap, gamepad1, gamepad2, telemetry, catchAndReleaseCommand);
        robotController = new GamepadController(hardwareMap, gamepad1, gamepad2, telemetry, true); // TeleopCommand(gamepad) class functions

        waitForStart();

        while (opModeIsActive())
        {
            try {
                robotController.update(true);

                catchAndReleaseCommand.runCommand();
                manualFixingCommand.runCommand();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}