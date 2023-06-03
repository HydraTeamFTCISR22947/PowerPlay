package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Transfer;

public class TransferCommand extends CommandBase {

    Transfer transfer ;

    HardwareMap hW;

    double pose;

    public TransferCommand ( Transfer transfer , HardwareMap hW, double pose)
    {

        this.transfer = transfer;
        this.hW = hW;
        this.pose = pose;




    }

    @Override
    public void initialize ()
    {




    }

    @Override
    public void execute (){


        transfer.goToPos((int) pose);




    }




}
