package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.gamepadHelper;

@TeleOp(name="Drivetrain Test", group="Tests")
public class teleop extends LinearOpMode {

  RobotCommands robotCommands;
  gamepadHelper gamepadOneHelper;

  @Override
  public void runOpMode()
  {
    robotCommands = new RobotCommands(hardwareMap, gamepad1);

    telemetry.addData("Status: ","Initialized!");
    telemetry.update();

    waitForStart();

    while (opModeIsActive())
    {
      robotCommands.drivetrainUpdate();
    }

  }


}
