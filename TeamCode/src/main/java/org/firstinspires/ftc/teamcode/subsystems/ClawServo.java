package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawServo {

    public static double closePos = 0.06; // catching angle of claws
    public static double openPos = 0.25; // releasing angle of claws

    private Servo _clawServoLeft; // declaring claw servo
    private Servo _clawServoRight; // declaring claw servo

    // constructor with the hardwaremap(the way the driver station knows to tell the code where each motor/servo is plugged.
    public ClawServo(HardwareMap hardwareMap)
    {
        // check the app for 2 servos with the names down below
        _clawServoLeft = hardwareMap.get(Servo.class,"claw_servo_left"); // initialize the claw
        _clawServoRight = hardwareMap.get(Servo.class,"claw_servo_right"); // initialize the claw

        // same as setting posiiton 1 - pos(reverse does 1 - pos) - in other words 0.1 in reverse is 0.9(1 is max - 270 degrees, 0 min - 0 degrees)
        _clawServoRight.setDirection(Servo.Direction.REVERSE);
    }

    public void openClaw()
    {
        _clawServoLeft.setPosition(openPos); // open the claw
        _clawServoRight.setPosition(openPos); // open the claw
    }

    public void closeClaw()
    {
        _clawServoLeft.setPosition(closePos); // close the claw
        _clawServoRight.setPosition(closePos); // close the claw
    }

}
