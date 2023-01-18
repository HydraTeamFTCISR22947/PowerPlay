package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Blue Drive Testing", group = "test")
public class DriveTestBlue extends LinearOpMode {

    Mecanum gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new Mecanum(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        gamepad.setRedAlliance(false);

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {
            gamepad.update();
        }

    }


}