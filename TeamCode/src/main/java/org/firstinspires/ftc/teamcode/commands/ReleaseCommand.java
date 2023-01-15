//package org.firstinspires.ftc.teamcode.commands;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
//import org.firstinspires.ftc.teamcode.subsystems.GripperSystem;
//import org.firstinspires.ftc.teamcode.subsystems.RotationServo;
//import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
//import org.firstinspires.ftc.teamcode.util.GamepadHelper;
//import org.firstinspires.ftc.teamcode.util.RobotCommand;
//
//public class ReleaseCommand implements RobotCommand {
//    TransferSystem transferSystem;
//    RotationServo rotationServo;
//    ClawServo clawServo;
//    GripperSystem gripperSystem;
//    Gamepad gamepad1;
//    GamepadHelper gamepadHelper1;
//
//    double halfwayOffset = 0;
//    double transferDelay = 3;
//    double finalOffset = 0;
//    double releaseDelay = 1;
//    double gripperOffset = 0;
//    double gripperDelay = 1;
//
//    public enum CatchingState {
//        OPEN_GRIPPER,
//
//        CATCH,
//        LIFT_HALF_WAY,
//        ROTATE,
//        FINISH_LIFT,
//        RELEASE_CONE,
//        CLOSE_GRIPPER,
//    };
//
//    CatchingState catchingState = CatchingState.RESET_CATCH;
//    ElapsedTime timer;
//
//    @Override
//    public void runCommand() {
//        switch (catchingState)
//        {
//            case RESET_CATCH:
//                transferSystem.setTransferLevel(TransferSystem.TransferLevels.ZERO);
//                transferSystem.update();
//
//                rotationServo.rotateClawForward();
//
//                clawServo.openClaw();
//
//                gripperSystem.openGripper();
//
//                if(gamepadHelper1.rightBumperOnce())
//                {
//                    catchingState = CatchingState.CATCH;
//                }
//                break;
//            case CATCH:
//                clawServo.closeClaw();
//
//                catchingState = CatchingState.LIFT_HALF_WAY;
//                break;
//            case LIFT_HALF_WAY:
//                transferSystem.setTransferLevel(TransferSystem.TransferLevels.MID);
//                transferSystem.update();
//
//                halfwayOffset = timer.time();
//                catchingState = CatchingState.ROTATE;
//                break;
//            case ROTATE:
//                if(timer.time() - halfwayOffset >= transferDelay)
//                {
//                    rotationServo.rotateClawBackward();
//
//                    catchingState = CatchingState.FINISH_LIFT;
//                }
//                break;
//            case FINISH_LIFT:
//                transferSystem.setTransferLevel(TransferSystem.TransferLevels.FINAL);
//                transferSystem.update();
//
//                finalOffset = timer.time();
//                break;
//            case RELEASE_CONE:
//                if(timer.time() - finalOffset >= releaseDelay)
//                {
//                    clawServo.openClaw();
//
//                    gripperOffset = timer.time();
//
//                    catchingState = CatchingState.FINISH_LIFT;
//                }
//                break;
//            case CLOSE_GRIPPER:
//                if(timer.time() - gripperOffset >= gripperDelay)
//                {
//                    gripperSystem.closeGripper();
//
//                    if(gamepadHelper1.leftBumperOnce())
//                    {
//                        catchingState = CatchingState.RESET_CATCH;
//                    }
//                }
//                break;
//        }
//    }
//
//    @Override
//    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1) {
//        transferSystem = new TransferSystem(hardwareMap);
//        rotationServo = new RotationServo(hardwareMap);
//        clawServo = new ClawServo(hardwareMap);
//        gripperSystem = new GripperSystem(hardwareMap);
//        timer =  new ElapsedTime();
//        this.gamepad1 = gamepad1;
//        gamepadHelper1 = new GamepadHelper(gamepad1);
//    }
//
//    void resetTimer()
//    {
//        timer.reset();
//    }
//}
