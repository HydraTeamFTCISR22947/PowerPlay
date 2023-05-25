package org.firstinspires.ftc.teamcode;
//Packages
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//Imports
public class ElevatorTester extends OpMode
{
    private ElevatorTester elevator;

    public void init()
    {
        elevator=new ElevatorTester(hardwareMap);
        elevator.init();
        //Initialize the elevator system with hardware mapping
    }

    public void start()
    {
        elevator.reset();
        //Function that reset the elevator to its initial position
    }

    public void loop()
    {
        double power = 0;
        if (gamepad1.dpad_up)
        {
            elevator.moveUp(power);
        }
        else if (gamepad1.dpad_down)
        {
            elevator.moveDown(power);
        }
        else
        {
            elevator.stop();
        }
        //Power to apply to the elevator(adjust as needed),
    }

    public void stop()
    {
        elevator.stop();
        //Function that stop the Elevator
    }
}
