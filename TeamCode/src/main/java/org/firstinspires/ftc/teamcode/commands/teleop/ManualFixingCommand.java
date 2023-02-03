package org.firstinspires.ftc.teamcode.commands.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
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


    public ManualFixingCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, CatchAndReleaseCommand catchAndReleaseCommand)
    {
        this.catchAndReleaseCommand = catchAndReleaseCommand;
        initCommand(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void runCommand() {
        gamepadHelper1.update();
        gamepadHelper2.update();

        controlOpenedElevator();

        userWantsBaseElevatorControl();
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

    void userWantsBaseElevatorControl()
    {
        double y = -gamepad2.left_stick_y;

        if(y != 0 && elevatorSystem.getLiftState() == ElevatorSystem.elevatorState.BASE_LEVEL)
        {
            elevatorSystem.setUsePID(false);
            elevatorSystem.setHeightByPos(elevatorSystem.currentPos());
        }
        else if(elevatorSystem.getLiftState() == ElevatorSystem.elevatorState.BASE_LEVEL && y == 0)
        {
            elevatorSystem.setUsePID(true);
        }

        if(gamepadHelper2.rightBumperOnce() && elevatorSystem.getLiftState() == ElevatorSystem.elevatorState.BASE_LEVEL)
        {
            elevatorSystem.setHeightByPos(elevatorSystem.currentPos() + elevatorSystem.INCREMENT);
        }
        else if(gamepadHelper2.leftBumperOnce() && elevatorSystem.getLiftState() == ElevatorSystem.elevatorState.BASE_LEVEL)
        {
            elevatorSystem.setHeightByPos(elevatorSystem.currentPos() - elevatorSystem.INCREMENT);
        }

    }

    void controlOpenedElevator()
    {
        if(catchAndReleaseCommand.isOpen)
        {
            double y = -gamepad2.left_stick_y;

            if(gamepadHelper2.rightBumperOnce())
            {
                elevatorSystem.setHeightByPos(elevatorSystem.currentPos() + elevatorSystem.INCREMENT);
            }
            else if(gamepadHelper2.leftBumperOnce())
            {
                elevatorSystem.setHeightByPos(elevatorSystem.currentPos() - elevatorSystem.INCREMENT);
            }

            if(y != 0)
            {
                elevatorSystem.setUsePID(false);
                elevatorSystem.setHeightByPos(elevatorSystem.currentPos());
            }
            else
            {
                elevatorSystem.setUsePID(true);
            }
        }
    }


}
