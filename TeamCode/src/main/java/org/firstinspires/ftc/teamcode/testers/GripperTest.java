package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.GripperSystem;

@TeleOp(name="Gripper Test", group="Tests")
public class GripperTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        GripperSystem gripperSystem = new GripperSystem(hardwareMap);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad1.left_bumper)
            {
                gripperSystem.openGripper();
            }
            else if(gamepad1.right_bumper)
            {
                gripperSystem.closeGripper();
            }
        }
    }
}
