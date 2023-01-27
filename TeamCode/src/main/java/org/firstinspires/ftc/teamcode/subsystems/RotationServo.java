package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RotationServo {

    public static double halfRotationForward = 0.55; // declare angle of rotation, putting the cone on platform
    public static double halfRotationBackward = 0; // declare angle of rotation, facing the next cone

    private Servo _rotationServo; // declare rotation servo

    public RotationServo(HardwareMap hardwareMap){
        _rotationServo = hardwareMap.get(Servo.class,"rotation_servo");
        rotateClawForward();
    }

    public void rotateClawForward(){
        _rotationServo.setPosition(halfRotationForward); // rotate for putting cone on platform

    }
    public void rotateClawBackward(){
        _rotationServo.setPosition(halfRotationBackward); // rotate to face next cone

    }


}
