package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@TeleOp(name="Field Centric Test", group="Tests")
public class FieldCentricTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        DcMotor mFL = hardwareMap.get(DcMotor.class, "mFL");
        DcMotor mFR = hardwareMap.get(DcMotor.class, "mFR");
        DcMotor mBL = hardwareMap.get(DcMotor.class, "mBL");
        DcMotor mBR = hardwareMap.get(DcMotor.class, "mBR");

        mBL.setDirection(DcMotorSimple.Direction.REVERSE);
        mFL.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(new Pose2d(0,0,-(Math.PI/2)));
        telemetry.addData("Status: ","Initialized!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x
            ).rotated(drivetrain.getExternalHeading());

            double twist = -gamepad1.right_stick_x;

            mFL.setPower(Range.clip(input.getX() + twist + input.getY(), -.5, .5));
            mBL.setPower(Range.clip(input.getX() + twist - input.getY(), -.5, .5));
            mFR.setPower(Range.clip(input.getX() - twist - input.getY(), -.5, .5));
            mBR.setPower(Range.clip(input.getX() - twist + input.getY(), -.5, .5));

            telemetry.addData("MFL", mFL.getCurrentPosition());
            telemetry.addData("MBL", mBL.getCurrentPosition());
            telemetry.addData("MFR", mFR.getCurrentPosition());
            telemetry.addData("MBR", mBR.getCurrentPosition());
            telemetry.addData("IMU", Math.toDegrees(drivetrain.getExternalHeading()));
            telemetry.update();
        }

    }
}
