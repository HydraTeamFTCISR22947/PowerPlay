package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.AutoRobotCommand;

@Config
public class AutoCommands implements AutoRobotCommand {
    ElevatorSystem elevatorSystem;
    TransferSystem transferSystem;
    RotationServo rotationServo;
    ClawServo clawServo;

    public static int STACK_ELEVATOR_HEIGHT = 710, OFFSET_BETWEEN_EACH_CONE = 40;

    public MarkerCallback catchCone() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                clawServo.closeClaw();
            }
        };
    }

    public MarkerCallback intakeFirstCone() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevatorSystem.goToPos(STACK_ELEVATOR_HEIGHT);
                transferSystem.pickUpExpansion();
                rotationServo.pickUpPosExpansion();
            }
        };
    }

    public MarkerCallback intakeSecondCone() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevatorSystem.goToPos(STACK_ELEVATOR_HEIGHT - OFFSET_BETWEEN_EACH_CONE * 2);
                transferSystem.pickUpExpansion();
                rotationServo.pickUpPosExpansion();
            }
        };
    }

    public MarkerCallback intakeThirdCone() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevatorSystem.goToPos(STACK_ELEVATOR_HEIGHT - OFFSET_BETWEEN_EACH_CONE * 3);
                transferSystem.pickUpExpansion();
                rotationServo.pickUpPosExpansion();
            }
        };
    }


    public MarkerCallback readyToRelease() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                transferSystem.highPos();
                elevatorSystem.midRod();
                rotationServo.releasePos();
            }
        };
    }

    public MarkerCallback elevatorIntake() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevatorSystem.midRod();
            }
        };
    }


    public MarkerCallback releaseCone() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                clawServo.openClaw();
            }
        };
    }


    public AutoCommands(HardwareMap hardwareMap) {
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
