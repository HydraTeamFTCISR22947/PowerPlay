package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;

@Config
@TeleOp(name="Servo Test", group="Tests")
public class ServoTest extends LinearOpMode {

    public static double position = 0;

    @Override
    public void runOpMode()
    {
        Servo servo = hardwareMap.get(Servo.class, "testServo");

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            servo.setPosition(position);
        }
    }
}
