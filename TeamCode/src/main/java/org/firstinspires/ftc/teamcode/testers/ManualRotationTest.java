package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Manual Rotation Test", group="Tests")
public class ManualRotationTest extends LinearOpMode {
    Servo _rotationServo;

    public static double pos = 0;

    @Override
    public void runOpMode()
    {
        _rotationServo = hardwareMap.get(Servo.class,"rotation_servo");
        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            _rotationServo.setPosition(pos);
        }
    }
}
