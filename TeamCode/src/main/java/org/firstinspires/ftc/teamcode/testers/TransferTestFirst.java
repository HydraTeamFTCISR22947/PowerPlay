package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystemFirst;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name="Transfer Test First", group="Tests")
public class TransferTestFirst extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        TransferSystemFirst transferSystem = new TransferSystemFirst(hardwareMap);
        GamepadHelper gamepadHelper1 = new GamepadHelper(gamepad1);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            transferSystem.update();

           // if (gamepadHelper1.YOnce()) {
            gamepadHelper1.update();
             if(gamepadHelper1.YOnce()){
                transferSystem.setTransferLevel(TransferSystemFirst.TransferLevels.RELEASE);
            } else if (gamepadHelper1.BOnce()) {
                transferSystem.setTransferLevel(TransferSystemFirst.TransferLevels.MID);
            } else if (gamepadHelper1.AOnce()) {
                transferSystem.setTransferLevel(TransferSystemFirst.TransferLevels.PICK_UP);
            }
        }
    }
}
