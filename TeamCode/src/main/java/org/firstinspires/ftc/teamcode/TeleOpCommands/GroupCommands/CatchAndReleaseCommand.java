package org.firstinspires.ftc.teamcode.TeleOpCommands.GroupCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.SubSystems.RotationServo;
import org.firstinspires.ftc.teamcode.SubSystems.Transfer;

import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ClawCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.RotationServoCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.TransferCommand;

public class CatchAndReleaseCommand extends SequentialCommandGroup {

    ElevatorCommand elevatorCommand1;
    ClawCommand clawCommand;
    TransferCommand transferCommand;
    RotationServoCommand rotationServoCommand;

    ElevatorSystem elevatorSystem;
    Claw claw;
    Transfer transfer;
    RotationServo rotationServo;

    HardwareMap hW;

    public CatchAndReleaseCommand (ElevatorCommand elevatorCommand, ClawCommand clawCommand,
                                   TransferCommand transferCommand , HardwareMap hW, Claw claw ,
                                   RotationServo rotationServo, RotationServoCommand rotationServoCommand,
                                   ElevatorSystem elevatorSystem , Transfer transfer){

        this.elevatorCommand1 = elevatorCommand;
        this.clawCommand = clawCommand;
        this.transferCommand = transferCommand;
        this.rotationServoCommand = rotationServoCommand;

        this.elevatorSystem = elevatorSystem;
        this.transfer = transfer;
        this.claw = claw;

        this.hW = hW;

        addCommands( new RotationServoCommand(hW, rotationServo , false , false, false),
                     new ClawCommand(hW, claw , true),
                     new TransferCommand(transfer , hW , transfer.HIGH) ,
                     new ElevatorCommand(elevatorSystem, hW, elevatorSystem.HIGH_POS),
                     new RotationServoCommand(hW, rotationServo , false , true, false));





    }



}