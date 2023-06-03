package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExampleCRSCode {

    // declaring our variables

    int target;

    HardwareMap hardwareMap;

    CRServo crServoExample;

    // creates an object that includes in him functions , variables , and caucilations of his own.
    public ExampleCRSCode(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;
        //Declaring the CRservo were going to use

        crServoExample = hardwareMap.get(CRServo.class , "crServoExample");

        // resetting the CRS to his starting position.

        crServoExample.setPower(0);
    }

    //example functions that are useable later on in other files
    public void CRSInfo(Telemetry telemetry)
    {
        telemetry.addData("CRS Power:", crServoExample.getPower());
        telemetry.addData("Motor's Target:", target);
        telemetry.addData("Motor's direction:", crServoExample.getDirection());
        telemetry.update();
    }


    public void CRSmovement(int power)
    {
        crServoExample.setPower(power);
        crServoExample.getDirection();

    }
}
