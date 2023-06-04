package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Buttons {

    public Object but;
    HardwareMap hW;

    Gamepad gamepad1;
    Gamepad gamepad2;

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

    public Buttons(Gamepad gamepad1, Gamepad gamepad2 ,  Button buttonA1,  Button buttonB1 ,  Button buttonX1 ,  Button buttonY1,
                   Button buttonA2,  Button buttonB2 , Button buttonX2,  Button buttonY2, Button buttonDpadUp1 , Button buttonDpadDown1 ,
                   Button buttonDpadRight1 ,Button buttonDpadLeft1 , Button buttonDpadUp2 , Button buttonDpadDown2 ,
                   Button buttonDpadRight2 ,Button buttonDpadLeft2)
    {


        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.buttonA1 = buttonA1;
        this.buttonB1 = buttonB1;
        this.buttonX1 = buttonX1;
        this.buttonY1 = buttonY1;

        this.buttonA2 = buttonA2;
        this.buttonB2 = buttonB2;
        this.buttonX2 = buttonX2;
        this.buttonY2 = buttonY2;

        this.buttonDpadUp1 = buttonDpadUp1;
        this.buttonDpadDown1 = buttonDpadDown1;
        this.buttonDpadRight1 = buttonDpadRight1;
        this.buttonDpadLeft1 = buttonDpadLeft1;

        this.buttonDpadUp2 = buttonDpadUp2;
        this.buttonDpadDown2 = buttonDpadDown2;
        this.buttonDpadRight2 = buttonDpadRight2;
        this.buttonDpadLeft2 = buttonDpadLeft2;

         buttonA1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.A);
         buttonB1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.B);
         buttonY1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.Y);
        buttonX1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.X);

         buttonDpadUp1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.DPAD_UP);
         buttonDpadDown1= new GamepadButton(GamepadEX1 , GamepadKeys.Button.DPAD_DOWN);
         buttonDpadRight1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.DPAD_RIGHT);
         buttonDpadLeft1 = new GamepadButton(GamepadEX1 , GamepadKeys.Button.DPAD_LEFT);


         buttonA2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.A);
         buttonB2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.B);
         buttonY2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.Y);
         buttonX2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.X);

         buttonDpadUp2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_UP);
         buttonDpadDown2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_DOWN);
         buttonDpadRight2 = new GamepadButton(GamepadEX2 , GamepadKeys.Button.DPAD_RIGHT);
         buttonDpadLeft2 = new GamepadButton(GamepadEX2  , GamepadKeys.Button.DPAD_LEFT);







    }
}
