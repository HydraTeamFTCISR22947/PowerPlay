package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ClawServo;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name = "Level Adjustment", group = "Tests")
public class AutoConeIntakeLevelAdjustment extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        TransferSystem transferSystem = new TransferSystem(hardwareMap);
        ElevatorSystem elevatorSystem = new ElevatorSystem(hardwareMap);
        ClawServo clawServoSystem  = new ClawServo(hardwareMap);
        GamepadHelper gamepadHelper1 = new GamepadHelper(gamepad1);

        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            gamepadHelper1.update();
            if(gamepadHelper1.YOnce())
            {
                transferSystem.pickUpExpansion();
                elevatorSystem.goToPos(elevatorSystem.STACK_ELEVATOR_HEIGHT);
            }
            else if(gamepadHelper1.BOnce())
            {
                transferSystem.pickUpExpansion();
                elevatorSystem.goToPos(elevatorSystem.STACK_ELEVATOR_HEIGHT - elevatorSystem.OFFSET_BETWEEN_EACH_CONE);

            }
            else if(gamepadHelper1.AOnce())
            {
                transferSystem.pickUpExpansion();
                elevatorSystem.goToPos(elevatorSystem.STACK_ELEVATOR_HEIGHT - elevatorSystem.OFFSET_BETWEEN_EACH_CONE * 2);
            }
            else if(gamepadHelper1.dpadLeft())
            {
                clawServoSystem.closeClaw();
            }
            else if(gamepadHelper1.dpadRight())
            {
                clawServoSystem.openClaw();
            }
            telemetry.addData("Transfer pos", transferSystem.getMotor().getCurrentPosition());
            telemetry.addData("target", transferSystem.getTarget());
            telemetry.addData("target in ticks", transferSystem.degreesToEncoderTicks(transferSystem.getTarget()));
            telemetry.update();



            telemetry.addData("Elevator pos", elevatorSystem.currentPos());
            telemetry.update();
        }
    }
}
