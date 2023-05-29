package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.ElevatorSystem;

public class ElevatorExtendOrLowerCommand extends CommandBase {

    public double pose;
    public ElevatorSystem elevatorSystem;

    HardwareMap hW;


    public ElevatorExtendOrLowerCommand (ElevatorSystem elevatorSystem , HardwareMap hardwareMap , int pose) {

        this.elevatorSystem = elevatorSystem;
        this.hW = hardwareMap;
        this.pose = pose;



        addRequirements((Subsystem) elevatorSystem);
    }


    @Override
    public void initialize() {

        elevatorSystem.baseLevel();

    }

    public void execute(){

        elevatorSystem.goToPos((int) pose);

    }

    @Override
    public boolean isFinished() {

        return true;

    }



}
