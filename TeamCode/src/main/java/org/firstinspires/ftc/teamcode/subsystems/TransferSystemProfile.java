package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class TransferSystemProfile {
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;

    public static double SPOOL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (spool) speed / input (motor) speed

    public static double PICK_UP = 305;
    public static double MID = 160;
    public static double RELEASE = 10;

    public static PIDCoefficients PID = new PIDCoefficients(2, 0, 0);

    public static double MAX_VEL = 5; // in/s
    public static double MAX_ACCEL = 5; // in/s^2
    public static double MAX_JERK = 0; // in/s^3

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static DcMotorEx motor;
    public static MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredAngle = 0;
    public static int offset;
    public static PIDFController controller;
    public static double power;

    private static double encoderTicksToDegrees(int ticks) {
        return (360 * ticks) / (TICKS_PER_REV * GEAR_RATIO);
    }

    public static int degreesToEncoderTicks(double degrees)
    {
        return (int)Math.round((TICKS_PER_REV * GEAR_RATIO) / (360 / degrees));
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MAX_RPM;
    }

    public TransferSystemProfile(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "mE");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // note: if the elevator is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it with a gravity feedforward
        controller = new PIDFController(PID, kV, kA, kStatic);
        offset = motor.getCurrentPosition();
    }

    public boolean isBusy(boolean setTrue) {
        if(setTrue)
        {
            return true;
        }
        else
        {
            return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
        }
    }

    public void setAngle(double angle) {
        angle = Math.min(Math.max(0, angle), PICK_UP);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy(false) ? profile.get(time) : new MotionState(desiredAngle, 0, 0, 0);
        MotionState goal = new MotionState(angle, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );

        profileStartTime = clock.seconds();

        this.desiredAngle = angle;
    }

    public double getCurrentAngle() {
        return encoderTicksToDegrees(motor.getCurrentPosition() - offset);
    }

    public void update() {
        double currentAngle = getCurrentAngle();
        if (isBusy(false)) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentAngle, state.getV());
        } else {
            // just hold the position
            controller.setTargetPosition(desiredAngle);
            power = controller.update(currentAngle);
        }
        setPower(power);
    }

    public double getVelocity()
    {
        if (isBusy(false)) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            return state.getV();
        } else {
            return 0;
        }
    }

    public double getTargetVelocity()
    {
        return controller.getTargetVelocity();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }



}