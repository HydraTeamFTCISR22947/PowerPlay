package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RotationServo;

@TeleOp(name="Rotation Servo Test", group="Tests")
public class RotationServoTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        RotationServo rotationServo = new RotationServo(hardwareMap);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad1.right_bumper)
            {
                rotationServo.pickUpPos();
            }
            else if(gamepad1.left_bumper)
            {
                rotationServo.releasePos();
            }
        }
    }
}
