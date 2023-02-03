package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Red", group = "Main")
public class teleopRed extends LinearOpMode {
    TeleopCommand teleop;

    @Override
    public void runOpMode() throws InterruptedException {
        teleop = new TeleopCommand(hardwareMap, gamepad1, gamepad2, telemetry, true);

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