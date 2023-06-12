
package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SubSystems.ElevatorSystem;

public class ElevatorCommand extends CommandBase {

    public ElevatorSystem elevatorSystem;
    HardwareMap hW;
    ElevatorSystem.ElevatorStates elevatorStates;


    public ElevatorCommand(ElevatorSystem elevatorSystem, HardwareMap hardwareMap, ElevatorSystem.ElevatorStates elevatorStates ) {

        this.elevatorSystem = elevatorSystem;
        this.hW = hardwareMap;
        this.elevatorStates = elevatorStates;

        addRequirements(elevatorSystem);
    }


    @Override
    public void initialize() {

        elevatorSystem.baseLevel();

    }

    public void execute() {

        elevatorSystem.update();


    }

}
