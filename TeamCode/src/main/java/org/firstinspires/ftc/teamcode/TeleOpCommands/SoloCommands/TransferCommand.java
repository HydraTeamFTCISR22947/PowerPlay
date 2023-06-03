package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Transfer;

public class TransferCommand extends CommandBase {

    Transfer transfer ;

    HardwareMap hW;

    int pose;

    public TransferCommand (HardwareMap hW , Transfer transfer, int pose)
    {

        this.transfer = transfer;
        this.hW = hW;
        this.pose = pose;

        addRequirements(transfer);


    }

    @Override
    public void initialize ()
    {

        transfer.toPickupPos();


    }

    @Override
    public void execute (){


        transfer.goToPos(pose);








    }


    @Override
    public boolean isFinished ()
    {

        return true;

    }



}
