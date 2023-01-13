package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RotationServo {

    double halfRotationForward = 0.67; // declare angle of rotation, putting the cone on platform
    double halfRotationBackward = 0; // declare angle of rotation, facing the next cone

    private Servo _rotationServo; // declare rotation servo
    private HardwareMap _hardwareMap; // declare hardware map

    public RotationServo(HardwareMap hardwareMap){

        this._hardwareMap = hardwareMap;
        _rotationServo = hardwareMap.get(Servo.class,"rotation_servo");

    }

    public void rotateClawForward(){
        _rotationServo.setPosition(halfRotationForward); // rotate for putting cone on platform

    }
    public void rotateClawBackward(){
        _rotationServo.setPosition(halfRotationBackward); // rotate to face next cone

    }


}
