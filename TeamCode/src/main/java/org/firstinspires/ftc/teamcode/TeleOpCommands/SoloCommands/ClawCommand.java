package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class ClawCommand extends CommandBase {

    HardwareMap hW ;

    Claw claw ;

    boolean gripperClose = false;

    public ClawCommand (HardwareMap hardwareMap , Claw claw,) {

        this.claw = claw;
        this.hW = hardwareMap;


        addRequirements(claw);

    }






    }
}
