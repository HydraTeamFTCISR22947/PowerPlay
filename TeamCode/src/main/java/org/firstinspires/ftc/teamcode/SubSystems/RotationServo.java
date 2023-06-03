package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class RotationServo extends SubsystemBase {


    public static double releasePos = .8; // declare angle of rotation, putting the cone on platform
    public static double releasePosExpansion = .15; // declare angle of rotation, putting the cone on platform
    public static double pickupPos = .8; // declare angle of rotation, facing the next cone
    public static double pickupExpansionPos = .25; // declare angle of rotation, facing the next cone

    private Servo _rotationServo; // declare rotation servo

    boolean expansionOrNormal, pickupOrNot;

    public RotationServo(HardwareMap hardwareMap, boolean expansionOrNormal , boolean pickupOrNot) {
        _rotationServo = hardwareMap.get(Servo.class, "rotation_servo");
        _rotationServo.setDirection(Servo.Direction.REVERSE);

        this.expansionOrNormal = expansionOrNormal;
        this.pickupOrNot = pickupOrNot;
    }

    public void releasePos() {
        _rotationServo.setPosition(releasePos); // rotate for putting cone on platform

    }

    public void pickUpPos() {
        _rotationServo.setPosition(pickupPos); // rotate to face next cone

    }

    public void releasePosExpansion() {
        _rotationServo.setPosition(releasePosExpansion); // rotate for putting cone on platform

    }

    public void pickUpPosExpansion() {
        _rotationServo.setPosition(pickupExpansionPos); // rotate to face next cone

    }

    public void setPosition(double pos) {
        _rotationServo.setPosition(pos);
    }


    @Override
    public void periodic() {

        if (expansionOrNormal == true) {

            pickUpPosExpansion();


        } else if (expansionOrNormal == false) {
            pickUpPos();


        }

         if (pickupOrNot == true) {

            releasePosExpansion();

        } else if (pickupOrNot == false ) {





        }else if (pickupOrNot == false && expansionOrNormal == false)
        {

            releasePos();
        }

    }
}