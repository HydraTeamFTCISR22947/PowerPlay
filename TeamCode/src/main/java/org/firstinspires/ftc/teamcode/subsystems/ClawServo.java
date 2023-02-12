package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawServo {

    public static double closePos = 0.05; // starting angle of claws
    public static double openPos = 0.25; // closed angle of claws

    private Servo _clawServoLeft; // declaring claw servo
    private Servo _clawServoRight; // declaring claw servo

    // constructor
    //lil mish
    public ClawServo(HardwareMap hardwareMap){

        _clawServoLeft = hardwareMap.get(Servo.class,"claw_servo_left"); // initialize the claw
        _clawServoRight = hardwareMap.get(Servo.class,"claw_servo_right"); // initialize the claw
        _clawServoRight.setDirection(Servo.Direction.REVERSE);
    }
    public void openClaw(){
        // if error occurred , change the order of commands
        _clawServoLeft.setPosition(openPos); // open the claw
        _clawServoRight.setPosition(openPos); // open the claw


    }
    public void closeClaw(){
        // if error, change the order of commands
        _clawServoLeft.setPosition(closePos); // close the claw
        _clawServoRight.setPosition(closePos); // close the claw

    }


}
