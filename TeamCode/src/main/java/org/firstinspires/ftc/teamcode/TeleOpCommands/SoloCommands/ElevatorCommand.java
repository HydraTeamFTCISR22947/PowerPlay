
package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.ElevatorCode;
import org.firstinspires.ftc.teamcode.SubSystems.ElevatorSystem;

public class ElevatorCommand extends CommandBase {

    public ElevatorSystem elevatorSystem;

    HardwareMap hW;

    int pose;


    public ElevatorCommand(ElevatorSystem elevatorSystem, HardwareMap hardwareMap, int pose) {

        this.elevatorSystem = elevatorSystem;
        this.hW = hardwareMap;
        this.pose = pose;

        addRequirements(elevatorSystem);
    }


    @Override
    public void initialize() {

        elevatorSystem.baseLevel();

    }

    public void execute() {

        elevatorSystem.goToPos(pose);


    }

}
