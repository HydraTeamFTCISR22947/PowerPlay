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

    // TODO: STACK
    public static int STACK_ELEVATOR_HEIGHT = 440, OFFSET_BETWEEN_EACH_CONE = 55;

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

    public MarkerCallback intakeFirstConeOpposite() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevatorSystem.goToPos(STACK_ELEVATOR_HEIGHT);
                transferSystem.pickUp();
                rotationServo.pickUpPos();
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

    public MarkerCallback intakeSecondConeOpposite() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevatorSystem.goToPos(STACK_ELEVATOR_HEIGHT - OFFSET_BETWEEN_EACH_CONE * 2);
                transferSystem.pickUp();
                rotationServo.pickUpPos();
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

    public MarkerCallback intakeThirdConeOpposite() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevatorSystem.goToPos(STACK_ELEVATOR_HEIGHT - OFFSET_BETWEEN_EACH_CONE * 3);
                transferSystem.pickUp();
                rotationServo.pickUpPos();
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

    public MarkerCallback readyToReleaseOpposite() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                transferSystem.highExpansionPos();
                elevatorSystem.midRod();
                rotationServo.releasePosExpansion();
            }
        };
    }

    public MarkerCallback reset() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevatorSystem.baseLevel();
                transferSystem.pickUp();
            }
        };
    }

    public MarkerCallback resetOpposite() {
        return new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevatorSystem.baseLevel();
                transferSystem.pickUpExpansion();
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

    public MarkerCallback goDownToReleaseCone() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevatorSystem.almostMidRod();
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
