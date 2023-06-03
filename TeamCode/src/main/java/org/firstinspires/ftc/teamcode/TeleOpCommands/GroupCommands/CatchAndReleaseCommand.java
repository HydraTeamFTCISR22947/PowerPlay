package org.firstinspires.ftc.teamcode.TeleOpCommands.GroupCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.SubSystems.Transfer;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ClawCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ElevatorExtendOrLowerCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.TransferCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.gripCommand;

public class CatchAndReleaseCommand extends SequentialCommandGroup {

    gripCommand gripCommand;
    ElevatorExtendOrLowerCommand elevatorExtendOrLowerCommand;
    ClawCommand clawCommand;
    TransferCommand transferCommand;

    ElevatorSystem elevatorSystem;
    Gripper gripper;
    Claw claw;
    Transfer transfer;

    HardwareMap hW;

    public CatchAndReleaseCommand ( gripCommand gripCommand , ElevatorExtendOrLowerCommand elevatorExtendOrLowerCommand , ClawCommand clawCommand,
                                    TransferCommand transferCommand ,HardwareMap hW, Claw claw ,ElevatorSystem elevatorSystem ,  Claw claw, Transfer transfer){

        this.gripCommand = gripCommand;
        this.elevatorExtendOrLowerCommand = elevatorExtendOrLowerCommand;
        this.clawCommand = clawCommand;
        this.transferCommand = transferCommand;

        this.gripper = gripper;
        this.elevatorSystem = elevatorSystem;
        this.transfer = transfer;
        this.claw = claw;

        this.hW = hW;

        addCommands(new gripCommand(hW, gripper, true, new TransferCommand(transfer , hW , ))


        );

    }



}
