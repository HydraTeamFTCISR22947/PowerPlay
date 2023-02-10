package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@Config
public class GamepadController {

    Gamepad gamepad1, gamepad2;
    DcMotor mFL, mBL, mFR, mBR;
    Telemetry telemetry;
    double power = mainPower;
    public static double mainPower = .4, multiplier = .9, POWER_INCREMENT = 0.1, slowPower = .3, fastPower = .65;
    GamepadHelper cGamepad1, cGamepad2;
    SampleMecanumDrive drivetrain;

    /**
     * constructor for gamepad
     * @param gamepad1 the gamepad1 object from TeleopCommand
     * @param gamepad2 the gamepad2 object from TeleopCommand
     * @param telemetry the telemetry object from TeleopCommand
     * //@param drivetrain the SampleMecanumDriveCancable object from TeleopCommand
     */
    public GamepadController(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean redAlliance) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.mFL = hardwareMap.get(DcMotor.class, "mFL");
        this.mFR = hardwareMap.get(DcMotor.class, "mFR");
        this.mBL = hardwareMap.get(DcMotor.class, "mBL");
        this.mBR = hardwareMap.get(DcMotor.class, "mBR");

        this.mBL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.mFL.setDirection(DcMotorSimple.Direction.REVERSE);

        this.drivetrain = new SampleMecanumDrive(hardwareMap);

        cGamepad1 = new GamepadHelper(gamepad1);
        cGamepad2 = new GamepadHelper(gamepad2);
        this.telemetry = telemetry;

        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We want to turn off velocity control for TeleopCommand
        // Velocity control per wheel is not necessary outside of motion profiled auto
        this.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(redAlliance)
        {
            this.drivetrain.setPoseEstimate(new Pose2d(0,0,-(Math.PI + Math.PI/2)));
        }
        else if(!redAlliance)
        {
            this.drivetrain.setPoseEstimate(new Pose2d(0,0,(Math.PI + Math.PI/2)));
        }
    }

    public void update(boolean redAlliance) {
        drivetrain.update();
        cGamepad1.update();
        cGamepad2.update();

        if(cGamepad1.XOnce())
        {
            if(redAlliance)
            {
                this.drivetrain.setPoseEstimate(new Pose2d(0,0,-(Math.PI + Math.PI/2)));
            }
            else if(!redAlliance)
            {
                this.drivetrain.setPoseEstimate(new Pose2d(0,0,(Math.PI + Math.PI/2)));
            }
        }

        if(cGamepad1.dpadUpOnce())
        {
            slowPower += POWER_INCREMENT;
            mainPower += POWER_INCREMENT;
            fastPower += POWER_INCREMENT;
        }
        else if(cGamepad1.dpadDownOnce())
        {
            slowPower -= POWER_INCREMENT;
            mainPower -= POWER_INCREMENT;
            fastPower -= POWER_INCREMENT;
        }

        centricDrive();
    }

    public void centricDrive()
    {
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x
        ).rotated(drivetrain.getExternalHeading());

        double twist = gamepad1.right_stick_x * multiplier;

        if(gamepad1.left_trigger != 0)
        {
            power = slowPower;
        }
        else if(gamepad1.right_trigger != 0)
        {
            power = fastPower;
        }
        else
        {
            power = mainPower;
        }

        mFL.setPower(Range.clip(input.getX() + twist + input.getY(), -power, power));
        mBL.setPower(Range.clip(input.getX() + twist - input.getY(), -power, power));
        mFR.setPower(Range.clip(input.getX() - twist - input.getY(), -power, power));
        mBR.setPower(Range.clip(input.getX() - twist + input.getY(), -power, power));
    }

    public void setPower(double power) {
        this.power = power;
    }

}