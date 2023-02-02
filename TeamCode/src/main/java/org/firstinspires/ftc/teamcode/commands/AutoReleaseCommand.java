package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.AutoRobotCommand;

public class AutoReleaseCommand implements AutoRobotCommand {
    static TransferSystem transferSystem;
    RotationServo rotationServo;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;
    ElapsedTime timer;

    public AutoReleaseCommand(HardwareMap hardwareMap)
    {
        initCommand(hardwareMap);
    }

    @Override
    public void runCommand()
    {
        clawServo.openClaw();


        double offset = timer.time();

        if(timer.time() - offset >= .5)
        {
            transferSystem.pickUp();
            elevatorSystem.baseLevel();
            rotationServo.pickUpPos();
            clawServo.openClaw();
        }
    }


    @Override
    public void initCommand(HardwareMap hardwareMap) {
        transferSystem = new TransferSystem(hardwareMap);
        rotationServo = new RotationServo(hardwareMap);
        clawServo = new ClawServo(hardwareMap);
        elevatorSystem = new ElevatorSystem(hardwareMap);
        timer =  new ElapsedTime();
    }

}
