package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

public class JunctionGuide {
    Servo servo = null;

    // position constants
    public static double DOWN = 0;
    public static double UP = 0.1667;

    // JunctionGuide constructor
    public JunctionGuide(HardwareMap map)
    {
        this.servo = map.get(Servo.class, "servo_guide");
    }

    // move guide up
    public void moveUp()
    {
        this.servo.setPosition(UP);
    }

    // move guide down
    public void moveDown()
    {
        this.servo.setPosition(DOWN);
    }


}
