package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class GripperSystem
{
    private Servo  _left , _right;
    public static double _openServoPos = 0;
    public static double _closeServoPos = 0.4;
    //in this part I write changes and for some of them I add values

    public GripperSystem(HardwareMap hardwaremap)
    {
        _left = hardwaremap.get(Servo.class, "gripper_servo_left");
        _right = hardwaremap.get(Servo.class, "gripper_servo_right");
        _right.setDirection(Servo.Direction.REVERSE);
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