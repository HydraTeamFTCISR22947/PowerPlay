package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name="Elevator Test", group="Tests")
public class ElevatorTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        ElevatorSystem elevatorSystem = new ElevatorSystem(hardwareMap);
        GamepadHelper gamepadHelper1 = new GamepadHelper(gamepad1);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            gamepadHelper1.update();
            if(gamepadHelper1.YOnce())
            {
                elevatorSystem.highRod();
            }
            else if(gamepadHelper1.BOnce())
            {
                elevatorSystem.midRod();
            }
            else if(gamepadHelper1.AOnce())
            {
                elevatorSystem.lowRod();
            }
            else if(gamepadHelper1.XOnce())
            {
                elevatorSystem.baseLevel();
            }
        }
    }
}
