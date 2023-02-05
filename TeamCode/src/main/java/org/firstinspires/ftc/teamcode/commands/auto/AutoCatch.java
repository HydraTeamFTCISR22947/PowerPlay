package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.AutoRobotCommand;

public class AutoCatch implements AutoRobotCommand {
    ElevatorSystem elevatorSystem;
    TransferSystem transferSystem;
    RotationServo rotationServo;
    ClawServo clawServo;

    public static int SECOND_STACK_ELEVATOR_HEIGHT = 0, OFFSET = 20;

    public MarkerCallback catchCone() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                clawServo.closeClaw();
            }
        };
    }

    public MarkerCallback intakeFirstCone() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevatorSystem.goToPos(SECOND_STACK_ELEVATOR_HEIGHT);
                transferSystem.pickUpExpansion();
                rotationServo.pickUpPos();
            }
        };
    }
    public MarkerCallback intakeSecondCone() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevatorSystem.goToPos(SECOND_STACK_ELEVATOR_HEIGHT + SECOND_STACK_ELEVATOR_HEIGHT * OFFSET);
                transferSystem.pickUpExpansion();
                rotationServo.pickUpPos();
            }
        };
    }

    public MarkerCallback intakeThirdCone() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevatorSystem.goToPos(SECOND_STACK_ELEVATOR_HEIGHT + OFFSET * 2);
                transferSystem.pickUpExpansion();
                rotationServo.pickUpPos();
            }
        };
    }


    public MarkerCallback readyToRelease() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                transferSystem.highExpansionPos();
                elevatorSystem.midRod();
                rotationServo.pickUpPos();
            }
        };
    }




    public AutoCatch(HardwareMap hardwareMap) {
        initCommand(hardwareMap);
    }

    @Override
    public void initCommand(HardwareMap hardwareMap) {
        elevatorSystem = new ElevatorSystem(hardwareMap);
        transferSystem = new TransferSystem(hardwareMap);
        rotationServo = new RotationServo(hardwareMap);
        clawServo = new ClawServo(hardwareMap);
        elevatorSystem = new ElevatorSystem(hardwareMap);
    }
}
