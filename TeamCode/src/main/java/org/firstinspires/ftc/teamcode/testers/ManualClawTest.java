package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Manual Claw Test", group="Tests")
public class ManualClawTest extends LinearOpMode {
    Servo _clawServoLeft, _clawServoRight;

    public static double pos1 = 0;
    public static double pos2 = 0;

    @Override
    public void runOpMode()
    {
        _clawServoLeft = hardwareMap.get(Servo.class,"claw_servo_left"); // initialize the claw
        _clawServoRight = hardwareMap.get(Servo.class,"claw_servo_right"); // initialize the claw
        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            _clawServoLeft.setPosition(pos1);
            _clawServoRight.setPosition(pos2);
        }
    }
}
