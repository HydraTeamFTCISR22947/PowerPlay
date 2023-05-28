package org.firstinspires.ftc.teamcode.TeleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Gripper;

public class gripCommand extends CommandBase {

    Gripper gripper ;

    HardwareMap hardwareMap;

    public  gripCommand (HardwareMap hW , Gripper gripper){

        this.hardwareMap = hW;
        this.gripper = gripper;


    }

    @Override
    public void initialize() {

        gripper.closeGripper();


    }

    public void execute (){

        gripper.openGripper();

    }
}
