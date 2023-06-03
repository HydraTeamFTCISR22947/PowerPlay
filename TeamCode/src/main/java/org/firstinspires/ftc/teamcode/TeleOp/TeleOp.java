package org.firstinspires.ftc.teamcode.TeleOp;

<<<<<<< HEAD
public class TeleOp {
}
=======
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ElevatorExtendOrLowerCommand;

public class TeleOp extends LinearOpMode {

    HardwareMap hW;

    ElevatorSystem elevatorSystem;
    ElevatorExtendOrLowerCommand elevatorExtendOrLowerCommand;

    GamepadEx GamepadEX1 = new GamepadEx(gamepad1);
    GamepadEx GamepadEX2 = new GamepadEx(gamepad2);

    Button buttonA1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.A);
    Button buttonB1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.B);
    Button buttonY1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.Y);
    Button buttonX1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.X);

    Button buttonDpadUp1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.DPAD_UP);
    Button buttonDpadDown1= new GamepadButton(GamepadEX1 , GamepadKeys.Button.DPAD_DOWN);
    Button buttonDpadRight1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.DPAD_RIGHT);
    Button buttonDpadLeft1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.DPAD_LEFT);


    Button buttonA2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.A);
    Button buttonB2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.B);
    Button buttonY2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.Y);
    Button buttonX2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.X);

    Button buttonDpadUp2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_UP);
    Button buttonDpadDown2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_DOWN);
    Button buttonDpadRight2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_RIGHT);
    Button buttonDpadLeft2 = new GamepadButton(GamepadEX2  , GamepadKeys.Button.DPAD_LEFT);



    public void runOpMode() throws InterruptedException {

        buttonY1.whenPressed( new ElevatorExtendOrLowerCommand(elevatorSystem, hardwareMap,elevatorSystem.HIGH_POS));
        buttonB1.whenPressed( new ElevatorExtendOrLowerCommand(elevatorSystem, hardwareMap,elevatorSystem.MID_POS));
        buttonA1.whenPressed( new ElevatorExtendOrLowerCommand(elevatorSystem, hardwareMap,elevatorSystem.LOW_POS));


    }
}





>>>>>>> parent of d127fda (k)
