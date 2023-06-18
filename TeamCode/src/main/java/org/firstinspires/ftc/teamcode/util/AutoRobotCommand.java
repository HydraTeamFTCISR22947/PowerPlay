package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface AutoRobotCommand {
    void initCommand(HardwareMap hardwareMap);
}
