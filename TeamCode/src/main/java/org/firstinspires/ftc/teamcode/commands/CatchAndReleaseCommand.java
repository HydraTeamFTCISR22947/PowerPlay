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
    static TransferSystem transferSystem;
    DriveCommand driveCommand;
    RotationServo rotationServo;
    ClawServo clawServo;
    ElevatorSystem elevatorSystem;
    Gamepad gamepad1, gamepad2;
    GamepadHelper gamepadHelper1;
    GamepadHelper gamepadHelper2;
    boolean pressed = false;
    double offset = 0;
    public static double delay = 0.5;
    boolean isOpen = false;

    public enum CatchingState {
        CATCH,
        UP,
        RESET_CATCH
    }

    enum LiftTarget
    {
        BASE,
        LOW,
        MID,
        HIGH
    }

    CatchingState catchingState = CatchingState.RESET_CATCH;
    LiftTarget liftTarget = LiftTarget.HIGH;
    ElapsedTime timer;

    public CatchAndReleaseCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, DriveCommand driveCommand)
    {
        this.driveCommand = driveCommand;

        initCommand(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void runCommand() throws Exception {
        switch (catchingState)
        {
            case RESET_CATCH:
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.PICK_UP);
                elevatorSystem.setLiftState(ElevatorSystem.elevatorState.BASE_LEVEL);
                rotationServo.pickUpPos();
                isOpen = false;
                clawServo.openClaw();

                if(gamepadHelper1.rightBumperOnce())
                {
                    catchingState = CatchingState.CATCH;
                }
                break;
            case CATCH:
                clawServo.closeClaw();

                if(gamepadHelper1.rightBumperOnce())
                {
                    rotationServo.releasePos();
                    catchingState = CatchingState.UP;
                }

                break;
            case UP:
                isOpen = true;
                transferSystem.setTransferLevel(TransferSystem.TransferLevels.HIGH);

                targetElevator();

                if(gamepadHelper1.leftBumperOnce())
                {
                    clawServo.openClaw();
                    saveCurrentPosElevator();
                    pressed = true;
                    offset = timer.time();
                }

                if(pressed)
                {
                    if(timer.time() - offset >= delay)
                    {
                        pressed = false;
                        catchingState = CatchingState.RESET_CATCH;
                    }
                }

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

        if(gamepadHelper1.leftBumperOnce() && catchingState != CatchingState.UP)
        {
            catchingState = CatchingState.RESET_CATCH;

        }

        elevatorSystem.update(gamepad2);
        transferSystem.update();
        gamepadHelper1.update();
        gamepadHelper2.update();
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
            case BASE:
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

    public static TransferSystem getTransferSystem() {
        return transferSystem;
    }

    public ElevatorSystem getElevatorSystem() {
        return elevatorSystem;
    }

    void saveCurrentPosElevator()
    {
        switch (liftTarget)
        {
            case LOW:
                elevatorSystem.setLowHeight(elevatorSystem.getPosition());
                break;
            case MID:
                elevatorSystem.setMidHeight(elevatorSystem.getPosition());
                break;
            case HIGH:
                elevatorSystem.setHighHeight(elevatorSystem.getPosition());
                break;
        }
    }
}
