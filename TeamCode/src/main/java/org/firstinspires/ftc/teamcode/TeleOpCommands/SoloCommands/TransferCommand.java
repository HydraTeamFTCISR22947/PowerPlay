package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Transfer;

public class TransferCommand extends CommandBase {

    Transfer transfer ;

    HardwareMap hW;

    public TransferCommand ( Transfer transfer , HardwareMap hW)
    {

        this.transfer = transfer;
        this.hW = hW;

        addRequirements(transfer);







    }

    @Override
    public void initialize ()
    {




    }

    @Override
    public void execute (){











    }




}
