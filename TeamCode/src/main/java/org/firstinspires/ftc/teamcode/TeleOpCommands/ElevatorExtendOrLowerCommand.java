package org.firstinspires.ftc.teamcode.TeleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.ElevatorCode;
import org.firstinspires.ftc.teamcode.SubSystems.ElevatorSystem;

public class ElevatorExtendOrLowerCommand extends CommandBase {

    public elevator elevatorSystem;

    HardwareMap hW;



    public ElevatorExtendOrLowerCommand (ElevatorSystem elevatorSystem , HardwareMap hardwareMap) {

        this.elevatorSystem = elevatorSystem;
        this.hW = hardwareMap;

        addRequirements((Subsystem) elevatorSystem);
    }


    @Override
    public void initialize() {

        elevatorSystem.baseLevel();

    }

    public void execute(){

        if()


    }



}
