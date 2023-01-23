package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.GripperSystem;

@Config
@TeleOp(name="Manual Gripper Test", group="Tests")
public class ManualGripperTest extends LinearOpMode {
    Servo _left, _right;

    public static double pos = 0;

    @Override
    public void runOpMode()
    {
        _left = hardwareMap.get(Servo.class, "gripper_servo_left");
        _right = hardwareMap.get(Servo.class, "gripper_servo_right");
        _right.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            _left.setPosition(pos);
            _right.setPosition(pos);
        }
    }
}
