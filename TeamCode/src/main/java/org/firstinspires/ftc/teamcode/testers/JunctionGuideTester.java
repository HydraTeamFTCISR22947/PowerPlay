package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.JunctionGuide;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@Config
@TeleOp(name="Junction Guide Tester", group="Tests")
public class JunctionGuideTester extends LinearOpMode
{
    JunctionGuide guide = new JunctionGuide(hardwareMap);
    GamepadHelper gamepadHelper1 = new GamepadHelper(gamepad1);

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while(opModeIsActive())
        {
            gamepadHelper1.update();
            if(gamepadHelper1.A())
            {
                guide.moveDown();
            }
            if(gamepadHelper1.B())
            {
                guide.moveUp();
            }
        }
    }
}