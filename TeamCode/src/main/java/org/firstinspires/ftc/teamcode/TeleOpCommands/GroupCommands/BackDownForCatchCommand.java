package org.firstinspires.ftc.teamcode.TeleOpCommands.GroupCommands;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.drive.SubSystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.drive.SubSystems.RotationServo;
import org.firstinspires.ftc.teamcode.drive.SubSystems.Transfer;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ClawCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.RotationServoCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.TransferCommand;

public class BackDownForCatchCommand extends SequentialCommandGroup {


    ElevatorCommand elevatorCommand1;
    ClawCommand clawCommand;
    TransferCommand transferCommand;
    RotationServoCommand rotationServoCommand;

    ElevatorSystem elevatorSystem;
    Claw claw;
    Transfer transfer;
    RotationServo rotationServo;


    public BackDownForCatchCommand(HardwareMap hW) {


        elevatorSystem = new ElevatorSystem(hW);
        transfer = new Transfer(hW);
        claw = new Claw(hW);
        rotationServo = new RotationServo(hW);

        addCommands(new ClawCommand(hW, claw, false),
                new RotationServoCommand(hW, rotationServo, false, false, false),
                new TransferCommand(transfer, hW, Transfer.TransferLevels.PICK_UP),
                new ElevatorCommand(elevatorSystem, hW, ElevatorSystem.ElevatorStates.LOW),
                new RotationServoCommand(hW, rotationServo, false, true, false));

        schedule();



    }
}
