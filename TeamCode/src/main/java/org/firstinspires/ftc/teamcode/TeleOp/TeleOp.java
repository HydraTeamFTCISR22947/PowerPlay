package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.drive.SubSystems.Transfer.HIGH;
import static org.firstinspires.ftc.teamcode.drive.SubSystems.Transfer.transferLevel;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.TeleOpCommands.GroupCommands.BackDownForCatchCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.GroupCommands.CatchAndReleaseCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.GroupCommands.LowRodScoreCommand;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "NewTeleOp", group = "Teleop")
@Config
public class TeleOp extends LinearOpMode {
    DriveTrain driveTrain;

    CatchAndReleaseCommand catchAndReleaseCommand;
    BackDownForCatchCommand backDownForCatchCommand;
    LowRodScoreCommand lowRodScoreCommand;

    GamepadEx GamepadEX1;
    GamepadEx GamepadEX2;

    Button buttonA1;
    Button buttonB1;
    Button buttonY1;
    Button buttonX1;

    Button buttonDpadUp1;
    Button buttonDpadDown1;
    Button buttonDpadRight1;
    Button buttonDpadLeft1;

/*
    Button buttonA2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.A);
    Button buttonB2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.B);
    Button buttonY2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.Y);
    Button buttonX2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.X);

    Button buttonDpadUp2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_UP);
    Button buttonDpadDown2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_DOWN);
    Button buttonDpadRight2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_RIGHT);
    Button buttonDpadLeft2 = new GamepadButton(GamepadEX2  , GamepadKeys.Button.DPAD_LEFT);

 */


    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEX1 = new GamepadEx(gamepad1);
        GamepadEX2 = new GamepadEx(gamepad2);

        buttonA1 = new GamepadButton(GamepadEX1, GamepadKeys.Button.A);
        buttonB1 = new GamepadButton(GamepadEX1, GamepadKeys.Button.B);
        buttonY1 = new GamepadButton(GamepadEX1, GamepadKeys.Button.Y);
        buttonX1 = new GamepadButton(GamepadEX1, GamepadKeys.Button.X);

        buttonDpadUp1 = new GamepadButton(GamepadEX1, GamepadKeys.Button.DPAD_UP);
        buttonDpadDown1 = new GamepadButton(GamepadEX1, GamepadKeys.Button.DPAD_DOWN);
        buttonDpadRight1 = new GamepadButton(GamepadEX1, GamepadKeys.Button.DPAD_RIGHT);
        buttonDpadLeft1 = new GamepadButton(GamepadEX1, GamepadKeys.Button.DPAD_LEFT);

        driveTrain = new DriveTrain(hardwareMap, true);
        catchAndReleaseCommand = new CatchAndReleaseCommand(hardwareMap);
        backDownForCatchCommand = new BackDownForCatchCommand(hardwareMap);
        lowRodScoreCommand = new LowRodScoreCommand(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            buttonY1.whenPressed(catchAndReleaseCommand, true);
            buttonA1.whenPressed(backDownForCatchCommand, true);
            buttonB1.whenPressed(lowRodScoreCommand, true);

            driveTrain.update(gamepad1);


            switch (transferLevel) {
                case PICK_UP:

                    transferLevel = transferLevel.PICK_UP;

                    break;

                case PICK_UP_EXPANSION:

                    if (gamepad1.dpad_left) {

                        transferLevel = transferLevel.PICK_UP_EXPANSION;

                    }
                    break;

                case HIGH:

                    if (gamepad1.dpad_up) {

                        transferLevel = transferLevel.HIGH;

                    }

                    break;

                case HIGH_EXPANSION:

                    if (gamepad1.dpad_down) {

                        transferLevel = transferLevel.HIGH_EXPANSION;
                    }

                    break;


            }
        }

    }


}











