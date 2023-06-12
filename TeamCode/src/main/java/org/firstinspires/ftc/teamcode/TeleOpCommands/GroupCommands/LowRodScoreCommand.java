package org.firstinspires.ftc.teamcode.TeleOpCommands.GroupCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.drive.SubSystems.RotationServo;
import org.firstinspires.ftc.teamcode.drive.SubSystems.Transfer;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.ClawCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.RotationServoCommand;
import org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands.TransferCommand;

public class LowRodScoreCommand extends SequentialCommandGroup {

    ClawCommand clawCommand;
    TransferCommand transferCommand;
    RotationServoCommand rotationServoCommand;

    Claw claw;
    Transfer transfer;
    RotationServo rotationServo;

    public LowRodScoreCommand(HardwareMap hW) {

        claw = new Claw(hW);
        transfer = new Transfer(hW);
        rotationServo = new RotationServo(hW);

        addCommands(new ClawCommand(hW, claw, true ),
                new RotationServoCommand(hW, rotationServo, false, true, false),
                new TransferCommand(transfer, hW, Transfer.TransferLevels.PICK_UP),
                new ClawCommand(hW, claw , false),
                new RotationServoCommand(hW, rotationServo, false, true, false));

        ;


    }
}
