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

    public MarkerCallback releaseCone() {
        return new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                clawServo.openClaw();
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
