package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystemProfile;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name="Transfer Test Profile", group="Tests")
public class TransferTestProfile extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        TransferSystemProfile transferSystem = new TransferSystemProfile(hardwareMap);
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
                transferSystem.setAngle(TransferSystemProfile.RELEASE);
            } else if (gamepadHelper1.BOnce()) {
                transferSystem.setAngle(TransferSystemProfile.MID);
            } else if (gamepadHelper1.AOnce()) {
                transferSystem.setAngle(TransferSystemProfile.PICK_UP);
            }
        }
    }
}
