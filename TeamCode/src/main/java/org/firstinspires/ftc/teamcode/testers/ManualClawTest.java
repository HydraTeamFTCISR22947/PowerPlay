package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Manual Claw Test", group="Tests")
public class ManualClawTest extends LinearOpMode {
    Servo _clawServo;

    public static double pos = 0;

    @Override
    public void runOpMode()
    {
        _clawServo = hardwareMap.get(Servo.class,"claw_servo"); // initialize the claw
        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            _clawServo.setPosition(pos);
        }
    }
}
