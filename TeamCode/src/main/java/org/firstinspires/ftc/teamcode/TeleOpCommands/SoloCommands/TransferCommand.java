package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SubSystems.Transfer;

public class TransferCommand extends CommandBase {

    Transfer transfer ;
    Transfer.TransferLevels transferLevels;

    HardwareMap hW;

    double pose;

    public TransferCommand ( Transfer transfer , HardwareMap hW, Transfer.TransferLevels transferLevels)
    {

        this.transfer = transfer;
        this.hW = hW;
        this.pose = pose;
        this.transferLevels = transferLevels;




    }

    @Override
    public void initialize ()
    {

        transferLevels =  transferLevels.PICK_UP;


    }

    @Override
    public void execute (){


        transfer.updateTransfer();




    }




}
