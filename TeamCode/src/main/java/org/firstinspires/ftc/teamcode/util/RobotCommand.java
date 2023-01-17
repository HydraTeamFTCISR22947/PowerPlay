package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface RobotCommand {
    public void runCommand();
    public void initCommand(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2);
}
