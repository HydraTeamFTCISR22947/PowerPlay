package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CatchAndReleaseCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp(name = "TeleOp", group = "Main")
public class teleop extends LinearOpMode {
    DriveCommand driveCommand;
    CatchAndReleaseCommand catchAndReleaseCommand;

    @Override
    public void runOpMode() throws InterruptedException {
        driveCommand = new DriveCommand(hardwareMap, gamepad1, gamepad2);
        catchAndReleaseCommand = new CatchAndReleaseCommand(hardwareMap, gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive())
        {
            driveCommand.runCommand();
            catchAndReleaseCommand.runCommand();
        }
    }
}