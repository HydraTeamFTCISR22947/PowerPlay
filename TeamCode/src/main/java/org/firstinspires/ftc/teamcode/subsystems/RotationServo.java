package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RotationServo {

    public static double releasePos = .8; // declare angle of rotation, putting the cone on platform
    public static double releasePosExpansion = .15; // declare angle of rotation, putting the cone on platform
    public static double pickupPos = .7; // declare angle of rotation, facing the next cone
    public static double pickupExpansionPos = .2; // declare angle of rotation, facing the next cone

    private Servo _rotationServo; // declare rotation servo

    public RotationServo(HardwareMap hardwareMap){
        _rotationServo = hardwareMap.get(Servo.class,"rotation_servo");
        _rotationServo.setDirection(Servo.Direction.REVERSE);
    }

    public void releasePos(){
        _rotationServo.setPosition(releasePos); // rotate for putting cone on platform

    }
    public void pickUpPos(){
        _rotationServo.setPosition(pickupPos); // rotate to face next cone

    }

    public void releasePosExpansion(){
        _rotationServo.setPosition(releasePosExpansion); // rotate for putting cone on platform

    }
    public void pickUpPosExpansion(){
        _rotationServo.setPosition(pickupExpansionPos); // rotate to face next cone

    }

    public void setPosition(double pos)
    {
        _rotationServo.setPosition(pos);
    }


}
