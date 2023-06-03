package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExampleMotorCode {

    // declaring our variables

    DcMotor motorExample;

    int target;

    HardwareMap hardwareMap;


    // creates an object that includes in him functions , variables , and caucilations of his own.
    public ExampleMotorCode(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;
        //Declaring the motors and servors were going to use
        motorExample = hardwareMap.get(DcMotor.class , "motorExample");

        //Reverse motor if needed , and declaring if motors are going to use their PID along with resetting motors them
        // to their starting positions.

        motorExample.setDirection(DcMotorSimple.Direction.REVERSE);

        motorExample.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExample.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExample.setTargetPosition(10);
        motorExample.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorExample.setPower(0);

    }

    //example functions that are useable later on in other files
    public void getMotorInfo(Telemetry telemetry)
    {
        telemetry.addData("Motor Position:", motorExample.getCurrentPosition());
        telemetry.addData("Motor's Target:", target);
        telemetry.addData("Motor's direction:", motorExample.getDirection());
        telemetry.update();
    }

    public void motorMovement(double power, int position)
    {
        motorExample.setTargetPosition(position);
        motorExample.setPower(power);
        motorExample.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
