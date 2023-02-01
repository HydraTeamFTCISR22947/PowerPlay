package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name="Transfer Test First", group="Tests")
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

            // if (gamepadHelper1.YOnce()) {
            gamepadHelper1.update();
            if(gamepadHelper1.YOnce())
            {
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.HIGH);
            }
            else if (gamepadHelper1.AOnce()) {
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.PICK_UP);
            }

             telemetry.addData("pos", transferSystem.getMotor().getCurrentPosition());
             telemetry.addData("target", transferSystem.getTarget());
             telemetry.addData("target in ticks", transferSystem.degreesToEncoderTicks(transferSystem.getTarget()));
             telemetry.update();
        }
    }
}
