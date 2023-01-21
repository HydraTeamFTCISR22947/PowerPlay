package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GamepadController;

@Config
@TeleOp(name = "Gamepad Red Test", group = "test")
public class TeleopBasicTest extends LinearOpMode {

    GamepadController gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new GamepadController(hardwareMap, gamepad1, gamepad2, telemetry); // TeleopCommand(gamepad) class functions
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