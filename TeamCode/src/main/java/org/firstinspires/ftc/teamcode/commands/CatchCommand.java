package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.RobotCommand;

public class CatchCommand implements RobotCommand {
    TransferSystem transferSystem;
    RotationServo rotationServo;
    ClawServo clawServo;

    public enum CatchingState {
        CATCH,
        LIFT_HALF_WAY,
        ROTATE,
        FINISH_LIFT,
        RELEASE_CONE,
        RESET_CATCH
    };

    CatchingState catchingState = CatchingState.RESET_CATCH;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runCommand() {
        switch (catchingState)
        {
            case RESET_CATCH:
                break;
            case CATCH:
                break;
            case LIFT_HALF_WAY:
                break;
            case ROTATE:
                break;
            case FINISH_LIFT:
                break;
            case RELEASE_CONE:
                break;
        }
    }

    @Override
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1) {
        transferSystem = new TransferSystem(hardwareMap);
        rotationServo = new RotationServo(hardwareMap);
        clawServo = new ClawServo(hardwareMap);
    }

    void resetTimer()
    {
        timer.reset();
    }
}
