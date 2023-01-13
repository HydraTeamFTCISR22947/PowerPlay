package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExampleServoCode {
    double target;
    HardwareMap hardwareMap;
    Servo servoExample;

    public ExampleServoCode(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;

        //Declaring the servo that were going to use
        servoExample = hardwareMap.get(Servo.class, "servoExample");
    }

    //example functions that are useable later on in other files
    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("Servo's angle:", servoExample.getPosition());
        telemetry.addData("Servos's Target:", target);
        telemetry.update();
    }

    public void moveServo(double position) {
        this.target = position;

        servoExample.setPosition(target);
    }

}