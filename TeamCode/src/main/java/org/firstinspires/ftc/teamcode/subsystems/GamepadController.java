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

    public boolean redAlliance = true;
    Gamepad gamepad1, gamepad2;
    DcMotor mFL, mBL, mFR, mBR;
    Telemetry telemetry;
    double drive,  strafe, twist, power = mainPower;
    public static double mainPower = .5, multiplier = .9, POWER_INCREMENT = 0.1;
    public static boolean slowTwist = true;
    GamepadHelper cGamepad1, cGamepad2;
    SampleMecanumDrive drivetrain;
    public static double startH = 0;

    /**
     * constructor for gamepad
     * @param gamepad1 the gamepad1 object from TeleopCommand
     * @param gamepad2 the gamepad2 object from TeleopCommand
     * @param telemetry the telemetry object from TeleopCommand
     * //@param drivetrain the SampleMecanumDriveCancable object from TeleopCommand
     */
    public GamepadController(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
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

        if(redAlliance)
        {
            this.drivetrain.setPoseEstimate(new Pose2d(0,0,(Math.PI/2)));

        }
        else
        {
            this.drivetrain.setPoseEstimate(new Pose2d(0,0,-(Math.PI/2)));
        }

        // We want to turn off velocity control for TeleopCommand
        // Velocity control per wheel is not necessary outside of motion profiled auto
        this.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        drivetrain.update();
        cGamepad1.update();
        cGamepad2.update();

        if(cGamepad1.XOnce())
        {
            if(redAlliance)
            {
                this.drivetrain.setPoseEstimate(new Pose2d(0,0,-(Math.PI/2)));
            }
            else if(!redAlliance)
            {
                this.drivetrain.setPoseEstimate(new Pose2d(0,0,(Math.PI/2)));
            }
        }

        if(cGamepad1.dpadUpOnce())
        {
            power += POWER_INCREMENT;
        }
        else if(cGamepad1.dpadDownOnce())
        {
            power -= POWER_INCREMENT;
        }

        getGamepadDirections();

        centricDrive();
    }

    public void getGamepadDirections()
    {
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;

        if(!slowTwist)
        {
            twist = -gamepad1.right_stick_x * multiplier;
        }
        else
        {
            twist = Range.clip(-gamepad1.right_stick_x * multiplier, -power/2, power/2);
        }

    }

    public void centricDrive()
    {
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x
        ).rotated(drivetrain.getExternalHeading());

        double twist = -gamepad1.right_stick_x;

        if(gamepad2.left_bumper)
        {
            power = 0.2;
        }
        else
        {
            power = .6;
        }

        mFL.setPower(Range.clip(input.getX() + twist + input.getY(), -power, power));
        mBL.setPower(Range.clip(input.getX() + twist - input.getY(), -power, power));
        mFR.setPower(Range.clip(input.getX() - twist - input.getY(), -power, power));
        mBR.setPower(Range.clip(input.getX() - twist + input.getY(), -power, power));
    }

    public void setRedAlliance(boolean redAlliance) {
        this.redAlliance = redAlliance;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public static void setSlowTwist(boolean slowTwist) {
        GamepadController.slowTwist = slowTwist;
    }
}