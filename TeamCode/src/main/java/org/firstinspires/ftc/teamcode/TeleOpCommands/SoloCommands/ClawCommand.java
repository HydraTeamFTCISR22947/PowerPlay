package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class ClawCommand extends CommandBase {

    HardwareMap hW;

    Claw claw;

    boolean close;

    boolean gripperClose = false;

    public ClawCommand(HardwareMap hardwareMap, Claw claw, boolean close) {

        this.claw = claw;
        this.hW = hardwareMap;
        this.close = close;

        addRequirements(claw);

    }


    public void initialize()
    {

        claw.closeClawClaw();



    }


    public void execute()
    {
        if(close == true)
        {
            claw.closeClawClaw();

        }else if(close == false){

            claw.openClaw();
        }

    }

    public boolean isFinished()
    {

        return true;
    }
}