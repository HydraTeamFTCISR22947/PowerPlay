package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    private Servo _left , _right;
    private HardwareMap _hardwaremap;
    private double _openServoPos = 0;
    private double _closeServoPos = 0;
    //in this part I write changes and for some of them I add values

    public Gripper(HardwareMap hardwaremap)
    {
        this._hardwaremap = hardwaremap;
        _left = hardwaremap.get(Servo.class, "gripper_servo_left");
        _right = hardwaremap.get(Servo.class, "gripper_servo_right");
        _left.setDirection(Servo.Direction.REVERSE);
        openGripper();
        //in this part I make fuction that change left servo direction,call other fuction and etc
    }
    public void openGripper()
    {
        _left.setPosition(_openServoPos);
        _right.setPosition(_openServoPos);
        //in this part I make fuction that open servos
    }
    public void closeGripper()
    {
        _left.setPosition(_closeServoPos);
        _right.setPosition(_closeServoPos);
        // in this part I make fuction that close servos
    }
}
