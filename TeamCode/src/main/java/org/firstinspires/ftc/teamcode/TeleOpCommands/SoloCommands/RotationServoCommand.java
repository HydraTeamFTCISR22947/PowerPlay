package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SubSystems.RotationServo;



public class RotationServoCommand extends CommandBase {

    RotationServo rotationServo;

    HardwareMap hW ;

    boolean expansionOrControl,  release, releaseExpansion;

    public RotationServoCommand (HardwareMap hW, RotationServo rotationServo , boolean expansionOrControl,  boolean release , boolean releaseExpansion)
    {

        this.hW = hW;
        this.rotationServo = rotationServo;

        this.expansionOrControl = expansionOrControl;
        this.release = release;
        this.releaseExpansion = releaseExpansion;

        addRequirements(rotationServo);

    }

    @Override
    public void initialize()
    {


        rotationServo.pickUpPos();



    }

    @Override
    public void execute ()
    {

        if (expansionOrControl == true) {

            rotationServo.pickUpPosExpansion();


        } else if (expansionOrControl == false) {

            rotationServo.pickUpPos();


        }

         if (release == true)
        {

            rotationServo.releasePos();

        }else if (releaseExpansion == true)
        {
            rotationServo.releasePosExpansion();
        }else {


         }

    }

    @Override
    public boolean isFinished ()
    {

        return true;

    }



}
