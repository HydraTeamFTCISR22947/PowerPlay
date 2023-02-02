package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RotationServo {

    public static double releasePos = 0.1; // declare angle of rotation, putting the cone on platform
    public static double pickupPos = 0.55; // declare angle of rotation, facing the next cone
    public static double pickupOppositePos = 1; // declare angle of rotation, facing the next cone

    private Servo _rotationServo; // declare rotation servo

    public RotationServo(HardwareMap hardwareMap){
        _rotationServo = hardwareMap.get(Servo.class,"rotation_servo");
    }

    public void releasePos(){
        _rotationServo.setPosition(releasePos); // rotate for putting cone on platform

    }
    public void pickUpPos(){
        _rotationServo.setPosition(pickupPos); // rotate to face next cone

    }

    public void setPosition(double pos)
    {
        _rotationServo.setPosition(pos);
    }


}
