package org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.PID;

@Config
public class VisionPID {
    public static PID.PIDCoefficients headingPID = new PID.PIDCoefficients( 0.001, 0, 0.005, 0.0001, 0);
    public static PID.PIDCoefficients drivePID = new PID.PIDCoefficients(0.007, 0, -0.005, 0.001, 0);

    static {
        headingPID.deadzone = 3;
        drivePID.deadzone = 5;
    }
}
