package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name="Transfer Test", group="Tests")
public class TransferTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        TransferSystem transferSystem = new TransferSystem(hardwareMap);
        GamepadHelper gamepadHelper1 = new GamepadHelper(gamepad1);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            transferSystem.update();

            if (gamepadHelper1.YOnce()) {
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.FINAL);
            } else if (gamepadHelper1.BOnce()) {
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.MID);
            } else if (gamepadHelper1.AOnce()) {
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.ZERO);
            }
        }
    }
}
