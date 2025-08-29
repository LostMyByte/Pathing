package org.firstinspires.ftc.teamcode.Utilities.Configuration;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Motion.Controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.DriveWheel;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;


public class DriveConfig {
    public static int DRIVE_MOTOR_MAX_CURRENT = 10000; // Maximum current a drivetrain motor should draw in milliAmps


    /*  Drive wheel Parameterization
        Each wheel is specified as a unitless 3D Vector, characterizing what direction the wheel moves in.
        Heading is included in the vector.
        See https://files.andymark.com/2008CON-Omni-Baker-McKenzie.pdf for details on math.

        Note that the Motor power unit is 1/(Battery level) of a volt. This could be useful for battery correction in the future.

        The trick is to separate u * (v_t + v_r) to u * v_t + u * v_r. Treat v_r as omega * leverarm, and then u * v_r becomes |u| * leverarm * omega.
        This can then be added to u * v_t to get the final dot product. If you then just put it in the vector, and have the third component be the
        angular velocity, it all works because v_r = leverarm * omega.
    */
    @Config
    public static class DriveWheels {
        public static double maxVelocity = 50;
        // BackEMF constants
        public static double Ex = 0.8;
        public static double Ey = 0.5;
        public static double Eh = 0;
        // Loopback constants
        public static double Lxk = 80;
        public static double Lyk = 50;
        public static double Lhk = 0;
        public static double Lxs = 90;
        public static double Lys = 60;
        public static double Lhs = 0;
        public static double Lmk = -0.07;
        public static double regimeChangeThreshold = 0.01;

        @Config
        public static class FL {
            public static double x = -1;
            public static double y = -1;
            public static double h = -1;

        }
        @Config
        public static class FR {
            public static double x = -1;
            public static double y = 1;
            public static double h = -1;

        }
        @Config
        public static class BL {
            public static double x = 1;
            public static double y = -1;
            public static double h = -1;

        }
        @Config
        public static class BR {
            public static double x = 1;
            public static double y = 1;
            public static double h = -1;

        }
        public static PIDCoefficients driveConstants = new PIDCoefficients(0.03, 0.01, 0, 0.03, 0.01);
        public static double driveAcceleration = 150;
    }









}
