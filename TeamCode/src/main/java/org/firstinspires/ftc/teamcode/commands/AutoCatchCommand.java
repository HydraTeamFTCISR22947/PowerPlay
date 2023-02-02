package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.AutoRobotCommand;

public class AutoCatchCommand implements AutoRobotCommand {
    TransferSystem transferSystem;
    RotationServo rotationServo;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;

    enum LiftTarget
    {
        BASE,
        LOW,
        MID,
        HIGH
    }

    enum StackTarget
    {
        FIFTH,
        FOURTH,
        THIRD,
        SECOND,
        FIRST
    }

    LiftTarget liftTarget = LiftTarget.HIGH;
    StackTarget stackTarget = StackTarget.FIFTH;
    ElapsedTime timer;

    public AutoCatchCommand(HardwareMap hardwareMap)
    {
        initCommand(hardwareMap);
    }



    @Override
    public void runCommand()
    {
        switch (stackTarget)
        {
            case FIFTH:
                elevatorSystem.goToPos(elevatorSystem.FIFTH_STACK_HEIGHT);
                break;
            case FOURTH:
                elevatorSystem.goToPos(elevatorSystem.FORTH_STACK_HEIGHT);
                break;
            case THIRD:
                elevatorSystem.goToPos(elevatorSystem.THIRD_STACK_HEIGHT);
                break;
            case SECOND:
                elevatorSystem.goToPos(elevatorSystem.SECOND_STACK_HEIGHT);
                break;
            case FIRST:
                elevatorSystem.goToPos(elevatorSystem.BASE_HEIGHT);
                break;
        }

        double offset = timer.time();

        if(timer.time() - offset >= 0.1)
        {
            clawServo.closeClaw();

            double secondOffset = timer.time();

            if(timer.time() - secondOffset >= 0.1)
            {
                targetElevator();

                rotationServo.releasePos();

                transferSystem.highPos();
            }
        }
    }

    void targetElevator()
    {
        switch (liftTarget)
        {
            case LOW:
                elevatorSystem.lowRod();
                break;
            case MID:
                elevatorSystem.midRod();
                break;
            case HIGH:
                elevatorSystem.highRod();
                break;
            case BASE:
                elevatorSystem.baseLevel();
                break;
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


    public void setLiftTarget(LiftTarget liftTarget) {
        this.liftTarget = liftTarget;
    }
}
