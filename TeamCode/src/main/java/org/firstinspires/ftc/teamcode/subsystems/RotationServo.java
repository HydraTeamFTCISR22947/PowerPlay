package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RotationServo {
    // default facing is control hub side

    public static double releasePos = .75; // declare angle of rotation, putting the cone on platform
    public static double releasePosExpansion = .25; // declare angle of rotation, putting the cone on platform but where transfer is at the expansion hub side
    public static double pickupPos = .8; // declare angle of rotation, facing the next cone to pick up
    public static double pickupExpansionPos = .2; // declare angle of rotation, facing the next cone to pickup but where transfer is at the expansion hub side

    private Servo _rotationServo; // declare rotation servo

    public RotationServo(HardwareMap hardwareMap)
    {
        _rotationServo = hardwareMap.get(Servo.class,"rotation_servo");

        // servo needs to be reversed
        _rotationServo.setDirection(Servo.Direction.REVERSE);
    }

    public void releasePos(){
        _rotationServo.setPosition(releasePos); // rotate for putting cone on platform

    }
    public void pickUpPos(){
        _rotationServo.setPosition(pickupPos); // rotate to face next cone

    }

    public void releasePosExpansion(){
        //_rotationServo.setPosition(releasePosExpansion); // rotate for putting cone on platform
        _rotationServo.setPosition(pickupPos); // rotate to face next cone

    }
    public void pickUpPosExpansion(){
        _rotationServo.setPosition(pickupExpansionPos); // rotate to face next cone

    }

    public void setPosition(double pos)
    {
        _rotationServo.setPosition(pos);
    }


}
