package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;
import org.firstinspires.ftc.teamcode.util.RobotCommand;

@Config
public class ManualFixingCommand implements RobotCommand {
    CatchAndReleaseCommand catchAndReleaseCommand;
    Gamepad gamepad1, gamepad2;
    GamepadHelper gamepadHelper1, gamepadHelper2;
    TransferSystem transferSystem;
    ElevatorSystem elevatorSystem;

   // double y = 0;

    public static double TRANSFER_INCREMENT = 5.5;

    public ManualFixingCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, CatchAndReleaseCommand catchAndReleaseCommand)
    {
        this.catchAndReleaseCommand = catchAndReleaseCommand;
        initCommand(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void runCommand() {
        gamepadHelper1.update();
        gamepadHelper2.update();

//        userWantsElevatorControl();
//
        switch (elevatorSystem.getLiftState())
        {
            case BASE_LEVEL:
                //elevatorSystem.setUsePID(true);
                fixBaseTransfer();
                break;
            case MID_ROD:
            case HIGH_ROD:
                //elevatorSystem.setUsePID(false);
                fixHighTransfer();
                break;
        }

        //saveCurrentPosElevator();


    }

    @Override
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.gamepadHelper1 = new GamepadHelper(gamepad1);
        this.gamepadHelper2 = new GamepadHelper(gamepad2);

        this.transferSystem = catchAndReleaseCommand.getTransferSystem();
        this.elevatorSystem = catchAndReleaseCommand.getElevatorSystem();
    }

    void fixBaseTransfer()
    {
        if(gamepadHelper2.rightBumperOnce())
        {
            transferSystem.setPickUp(transferSystem.getPickUp() - TRANSFER_INCREMENT);
        }
        else if(gamepadHelper2.leftBumperOnce())
        {
            transferSystem.setPickUp(transferSystem.getPickUp() + TRANSFER_INCREMENT);
        }
    }

    void fixHighTransfer()
    {
        if(gamepadHelper2.rightBumperOnce())
        {
            transferSystem.setHIGH(transferSystem.getHIGH() + TRANSFER_INCREMENT);
        }
        else if(gamepadHelper2.leftBumperOnce())
        {
            transferSystem.setHIGH(transferSystem.getHIGH() - TRANSFER_INCREMENT);
        }
    }

//    void saveCurrentPosElevator()
//    {
//        switch (catchAndReleaseCommand.getLiftTarget())
//        {
//            case MID:
//                elevatorSystem.setMidHeight(elevatorSystem.currentPos());
//                break;
//            case HIGH:
//                elevatorSystem.setHighHeight(elevatorSystem.currentPos());
//                break;
//        }
//    }

//    void userWantsElevatorControl()
//    {
//        y = -gamepad2.left_stick_y;
//
//        if(y != 0)
//        {
//            elevatorSystem.setUsePID(false);
//        }
//        else if(catchAndReleaseCommand.getLiftTarget() != CatchAndReleaseCommand.LiftTarget.MID && catchAndReleaseCommand.getLiftTarget() != CatchAndReleaseCommand.LiftTarget.HIGH)
//        {
//            elevatorSystem.setUsePID(true);
//        }
//    }


}
