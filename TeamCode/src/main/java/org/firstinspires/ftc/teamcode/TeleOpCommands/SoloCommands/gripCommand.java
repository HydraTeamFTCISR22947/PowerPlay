package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class gripCommand extends CommandBase {

    Gripper gripper;

    HardwareMap hardwareMap;

    boolean gripperClose;

    public gripCommand(HardwareMap hW, Gripper gripper, boolean gripperClose) {

        this.hardwareMap = hW;
        this.gripper = gripper;


    }

    @Override
    public void initialize() {

        gripper.closeGripper();


    }

    public void execute() {

        if (gripperClose == true) {

            gripper.closeGripper();


        } else if (gripperClose == false) {

            gripper.openGripper();

        }

    }
}
