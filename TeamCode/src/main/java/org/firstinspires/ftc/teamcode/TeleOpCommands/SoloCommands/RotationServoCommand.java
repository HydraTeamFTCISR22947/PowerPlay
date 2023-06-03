package org.firstinspires.ftc.teamcode.TeleOpCommands.SoloCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.RotationServo;



public class RotationServoCommand extends CommandBase {

    RotationServo rotationServo;

    HardwareMap hW ;

    boolean expansionOrNormal, pickupOrNot;

    public RotationServoCommand (HardwareMap hW, RotationServo rotationServo , boolean expansionOrNormal, boolean pickupOrNot)
    {

        this.hW = hW;
        this.rotationServo = rotationServo;

        this.expansionOrNormal = expansionOrNormal;
        this.pickupOrNot = pickupOrNot;

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

        if (expansionOrNormal == true) {

            rotationServo.pickUpPosExpansion();


        } else if (expansionOrNormal == false) {
            rotationServo.pickUpPos();


        }

        if (pickupOrNot == true) {

            rotationServo.releasePosExpansion();

        } else if (pickupOrNot == false ) {





        }else if (pickupOrNot == false && expansionOrNormal == false)
        {

            rotationServo.releasePos();
        }


    }

    @Override
    public boolean isFinished ()
    {

        return true;

    }



}
