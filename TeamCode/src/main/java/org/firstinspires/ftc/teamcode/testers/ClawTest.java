package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;

@TeleOp(name="Claw Test", group="Tests")
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        ClawServo clawServo = new ClawServo(hardwareMap);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad1.left_bumper)
            {
                clawServo.openClaw();
            }
            else if(gamepad1.right_bumper)
            {
                clawServo.closeClaw();
            }
        }
    }
}
