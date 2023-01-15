package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp - Gamepads", group="Tests")
public class teleop extends LinearOpMode {


  @Override
  public void runOpMode()
  {

    telemetry.addData("Status: ","Initialized!");
    telemetry.update();

    waitForStart();

    while (opModeIsActive())
    {
    }

  }


}
