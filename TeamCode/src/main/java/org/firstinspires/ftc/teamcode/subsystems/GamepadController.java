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
    public static double mainPower = .6, multiplier = .9, POWER_INCREMENT = 0.1, slowPower = .3, fastPower = .7;
    GamepadHelper cGamepad1, cGamepad2;
    SampleMecanumDrive drivetrain;

    /**
     * constructor for gamepad
     * @param gamepad1 the gamepad1 object from TeleopCommand
     * @param gamepad2 the gamepad2 object from TeleopCommand
     * @param telemetry the telemetry object from TeleopCommand
     * @param redAlliance check if code is started from red or blue alliance
     */
    public GamepadController(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean redAlliance) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.mFL = hardwareMap.get(DcMotor.class, "mFL");
        this.mFR = hardwareMap.get(DcMotor.class, "mFR");
        this.mBL = hardwareMap.get(DcMotor.class, "mBL");
        this.mBR = hardwareMap.get(DcMotor.class, "mBR");

        // reverse left motors
        this.mBL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.mFL.setDirection(DcMotorSimple.Direction.REVERSE);

        // get roadrunner drive(ask elior) for imu(angle sensor) of robot
        this.drivetrain = new SampleMecanumDrive(hardwareMap);

        // gamepad helper to see if pressed button once
        cGamepad1 = new GamepadHelper(gamepad1);
        cGamepad2 = new GamepadHelper(gamepad2);
        this.telemetry = telemetry;

        // brake motors
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We want to turn off velocity control for TeleopCommand
        // Velocity control per wheel is not necessary outside of motion profiled auto
        this.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // check if we are blue/red alliance and set zero angle
        if(!redAlliance)
        {
            this.drivetrain.setPoseEstimate(new Pose2d(0,0,-(Math.PI + Math.PI/2)));
        }
        else if(redAlliance)
        {
            this.drivetrain.setPoseEstimate(new Pose2d(0,0,(Math.PI + Math.PI/2)));
        }
    }

    public void update(boolean redAlliance) {
        // update imu angle and gamepad helpers
        drivetrain.update();
        cGamepad1.update();
        cGamepad2.update();

        // if pressed x so reset zero angle
        if(cGamepad1.XOnce())
        {
            if(!redAlliance)
            {
                this.drivetrain.setPoseEstimate(new Pose2d(0,0,-(Math.PI + Math.PI/2)));
            }
            else if(redAlliance)
            {
                this.drivetrain.setPoseEstimate(new Pose2d(0,0,(Math.PI + Math.PI/2)));
            }
        }

        // make power more/less
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

        // calculate motor power and set power
        centricDrive();
    }

    public void centricDrive()
    {
        // get the inputs and rotate the vector(cos and sin) by robot heading(angle)
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x
        ).rotated(drivetrain.getExternalHeading());

        // get the twist value for robot(spinning in place)
        double twist = gamepad1.right_stick_x * multiplier;

        // check if press right/left triggger for slow/fast power
        if(gamepad1.right_trigger != 0)
        {
            power = slowPower;
        }
        else if(gamepad1.left_trigger != 0)
        {
            power = fastPower;
        }
        else
        {
            power = mainPower;
        }

        // set power and clip it between max and min
        mFL.setPower(Range.clip(input.getX() + twist + input.getY(), -power, power));
        mBL.setPower(Range.clip(input.getX() + twist - input.getY(), -power, power));
        mFR.setPower(Range.clip(input.getX() - twist - input.getY(), -power, power));
        mBR.setPower(Range.clip(input.getX() - twist + input.getY(), -power, power));
    }

    public void setPower(double power) {
        this.power = power;
    }

}