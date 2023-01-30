package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;
import org.firstinspires.ftc.teamcode.util.RobotCommand;

public class CatchAndReleaseCommand implements RobotCommand {
    TransferSystem transferSystem;
    RotationServo rotationServo;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;
    Gamepad gamepad1, gamepad2;
    GamepadHelper gamepadHelper1;
    GamepadHelper gamepadHelper2;

    public enum CatchingState {
        CATCH,
        TRANSFER_HALF_WAY,
        ROTATE,
        FINISH_TRANSFER,
        LIFT_UP,
        RESET_CATCH
    }

    enum LiftTarget
    {
        LOW,
        MID,
        HIGH
    }

    CatchingState catchingState = CatchingState.RESET_CATCH;
    LiftTarget liftTarget = LiftTarget.HIGH;
    ElapsedTime timer;

    public CatchAndReleaseCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry)
    {
        initCommand(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void runCommand() throws Exception {
        switch (catchingState)
        {
            case RESET_CATCH:
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.PICK_UP);
                elevatorSystem.setLiftState(ElevatorSystem.elevatorState.BASE_LEVEL);
                rotationServo.rotateClawBackward();

                clawServo.openClaw();

                if(gamepadHelper1.rightBumperOnce())
                {
                    catchingState = CatchingState.CATCH;
                }
                break;
            case CATCH:
                clawServo.closeClaw();

                //if(gamepadHelper1.rightBumperOnce())
                //{
                    catchingState = CatchingState.TRANSFER_HALF_WAY;
                //}

                break;
            case TRANSFER_HALF_WAY:
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.MID);

                if(gamepadHelper1.rightBumperOnce())
                {
                    catchingState = CatchingState.ROTATE;
                }
                break;
            case ROTATE:
                rotationServo.rotateClawForward();

                catchingState = CatchingState.FINISH_TRANSFER;
                break;
            case FINISH_TRANSFER:
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.HIGH);

                catchingState = CatchingState.LIFT_UP;
                break;
            case LIFT_UP:
                rotationServo.rotateClawBackward();

                clawServo.openClaw();

                targetElevator();

                break;
        }

        if(gamepadHelper2.YOnce())
        {
            liftTarget = LiftTarget.HIGH;
        }
        else if(gamepadHelper2.BOnce())
        {
            liftTarget = LiftTarget.MID;
        }
        else if(gamepadHelper2.AOnce())
        {
            liftTarget = LiftTarget.LOW;
        }

        if(gamepadHelper1.leftBumperOnce())
        {
            //saveCurrentPos();
            catchingState = CatchingState.RESET_CATCH;
        }

        elevatorSystem.update(gamepad2);
        transferSystem.update();
        gamepadHelper1.update();
        gamepadHelper2.update();
    }

    void saveCurrentPosElevator()
    {
        switch (liftTarget)
        {
            case LOW:
                elevatorSystem.setLowHeight(elevatorSystem.currentPos());
                break;
            case MID:
                elevatorSystem.setMidHeight(elevatorSystem.currentPos());
                break;
            case HIGH:
                elevatorSystem.setHighHeight(elevatorSystem.currentPos());
                break;
        }
    }

    void saveCurrentPosTransfer()
    {
        transferSystem.setHIGH(transferSystem.getMotor().getCurrentPosition());
    }

    void targetElevator()
    {
        switch (liftTarget)
        {
            case LOW:
                elevatorSystem.setLiftState(ElevatorSystem.elevatorState.LOW_ROD);
                break;
            case MID:
                elevatorSystem.setLiftState(ElevatorSystem.elevatorState.MID_ROD);
                break;
            case HIGH:
                elevatorSystem.setLiftState(ElevatorSystem.elevatorState.HIGH_ROD);
                break;
        }

        elevatorSystem.update(gamepad2);
    }
    @Override
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        transferSystem = new TransferSystem(hardwareMap);
        rotationServo = new RotationServo(hardwareMap);
        clawServo = new ClawServo(hardwareMap);
        elevatorSystem = new ElevatorSystem(hardwareMap);
        timer =  new ElapsedTime();
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        gamepadHelper1 = new GamepadHelper(gamepad1);
        gamepadHelper2 = new GamepadHelper(gamepad2);
    }

}
