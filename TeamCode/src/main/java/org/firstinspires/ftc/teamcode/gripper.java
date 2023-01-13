package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class gripper
{
    private Servo  _left , _right;
    private HardwareMap _hardwaremap;
    private double _openServoPos = 0;
    private double _closeServoPos = 0;
    //בשלב עשינו משתנים ולכמה מיהם נתנו ערכים

    public gripper(HardwareMap hardwaremap)
    {
        this._hardwaremap = hardwaremap;
        _left = hardwaremap.get(Servo.class, "gripper_servo_left");
        _right = hardwaremap.get(Servo.class, "gripper_servo_right");
        _left.setDirection(Servo.Direction.REVERSE);
        openGripper();
    //בשלב זה יצרנו פונקציה שאחראית על לתת למשתנים
    }
    public void openGripper()
    {
        _left.setPosition(_openServoPos);
        _right.setPosition(_openServoPos);
    //
    }
    public void closeGripper()
    {
        _left.setPosition(_closeServoPos);
        _right.setPosition(_closeServoPos);
    // בשלב זה יצרנו עוד פונקציה שאחראית על סגירת הסרבואים
    }
}
