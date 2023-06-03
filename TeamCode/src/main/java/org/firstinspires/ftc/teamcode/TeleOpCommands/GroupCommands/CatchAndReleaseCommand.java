package org.firstinspires.ftc.teamcode.TeleOpCommands.GroupCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.SubSystems.RotationServo;
import org.firstinspires.ftc.teamcode.SubSystems.Transfer;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ClawCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ElevatorExtendOrLowerCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.TransferCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.rotationServoCommand;

public class CatchAndReleaseCommand extends SequentialCommandGroup {


    ElevatorExtendOrLowerCommand elevatorExtendOrLowerCommand;
    ClawCommand clawCommand;
    TransferCommand transferCommand;
    RotationServo rotationServo;

    ElevatorSystem elevatorSystem;
    Claw claw;
    Transfer transfer;

    HardwareMap hW;

    public CatchAndReleaseCommand (  HardwareMap hW, Claw claw ,ElevatorSystem elevatorSystem ,  Transfer transfer, RotationServo rotation){

        this.elevatorSystem = elevatorSystem;
        this.transfer = transfer;
        this.claw = claw;
        this.rotationServo = rotation;

        this.hW = hW;

        new SequentialCommandGroup(new ClawCommand(hW , claw) ,
                new rotationServoCommand(hW, rotationServo , true , false) ,
                new TransferCommand(hW, transfer , (int) transfer.HIGH),
                new ElevatorExtendOrLowerCommand(elevatorSystem, hW, elevatorSystem.HIGH_POS),
                new rotationServoCommand(hW , rotationServo , false , false ));

    }



}
