package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@Config
public class GamepadController {

    public boolean redAlliance = true;
    Gamepad gamepad1, gamepad2;
    DcMotor mFL, mBL, mFR, mBR;
    Telemetry telemetry;
    double leftPower_f;
    double leftPower_b;
    double rightPower_f;
    double rightPower_b;
    double drive,  strafe, twist, power = mainPower;
    public static double mainPower = .6, multiplier = .9, POWER_INCREMENT = 0.1;
    public static boolean slowMove = false, isCentricDrive = true, canTwist = true;
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
        this.mBL = hardwareMap.get(DcMotor.class, "mBL");
        this.mBR = hardwareMap.get(DcMotor.class, "mBR");
        this.mFR = hardwareMap.get(DcMotor.class, "mFR");
        this.mFL.setDirection(DcMotor.Direction.REVERSE);
        this.mBL.setDirection(DcMotor.Direction.REVERSE);
        cGamepad1 = new GamepadHelper(gamepad1);
        cGamepad2 = new GamepadHelper(gamepad2);
        this.telemetry = telemetry;
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(redAlliance)
        {
            startH = 0;
            startH -= (Math.PI + Math.PI/2); // red
        }
        else
        {
            startH = 0;
            startH -= Math.PI/2; // blue
        }

        this.drivetrain = new SampleMecanumDrive(hardwareMap);
        this.drivetrain.setPoseEstimate(new Pose2d(0,0,startH));
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
                startH = 0;
                startH -= (Math.PI + Math.PI/2); // red
                this.drivetrain.setPoseEstimate(new Pose2d(0,0,startH));
            }
            else if(!redAlliance)
            {
                startH = 0;
                startH -= Math.PI/2; // blue
                this.drivetrain.setPoseEstimate(new Pose2d(0,0,startH));
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

//        if(cGamepad1.dpadDownOnce())
//        {
//            isCentricDrive = !isCentricDrive;
//        }
//
        if (isCentricDrive)
        {
            centricDrive();
        }
        else
        {
            regularDrive();
        }

        mFL.setPower(leftPower_f);
        mBL.setPower(leftPower_b);
        mFR.setPower(rightPower_f);
        mBR.setPower(rightPower_b);
    }

    public void getGamepadDirections()
    {
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        if(canTwist)
        {
            twist = -gamepad1.right_stick_x * multiplier;
        }
        else
        {
            twist = 0;
        }
    }

    public void regularDrive()
    {
        leftPower_f = Range.clip(drive + twist + strafe, -power, power);
        leftPower_b = Range.clip(drive + twist - strafe, -power, power);
        rightPower_f = Range.clip(drive - twist - strafe, -power, power);
        rightPower_b = Range.clip(drive - twist + strafe, -power, power);
    }

    public void centricDrive()
    {
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x
        ).rotated(drivetrain.getExternalHeading());

        leftPower_f = Range.clip(input.getX() + twist + input.getY() , -power, power);
        leftPower_b = Range.clip(input.getX() + twist - input.getY(), -power, power);
        rightPower_f = Range.clip(input.getX() - twist - input.getY(), -power, power);
        rightPower_b = Range.clip(input.getX() - twist + input.getY(), -power, power);
    }

    public double getIMU()
    {
        return drivetrain.getExternalHeading();
    }

    public void saveIMUHeading()
    {
        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), "" + getIMU());
    }

    public void setSlowMove(boolean slowMove) {
        this.slowMove = slowMove;
    }

    public void setRedAlliance(boolean redAlliance) {
        this.redAlliance = redAlliance;
    }

    public static boolean isCanTwist() {
        return canTwist;
    }

    public void setCanTwist(boolean canTwist) {
        this.canTwist = canTwist;
    }

    public void setPower(double power) {
        this.power = power;
    }
}