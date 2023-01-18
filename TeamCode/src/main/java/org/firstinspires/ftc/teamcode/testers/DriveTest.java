package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Red Drive Testing", group = "test")
public class DriveTest extends LinearOpMode {

    Mecanum gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new Mecanum(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        gamepad.setRedAlliance(true);

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad2.right_bumper)
            {
                gamepad.setPower(.9);
            }
            else if(gamepad2.left_bumper)
            {
                gamepad.setPower(.2);
            }
            else
            {
                gamepad.setPower(.6);
            }
            gamepad.update();
        }

    }


}