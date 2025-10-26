// Primary Author: Kieran Mattingly, Mixed
package org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.MPCPath;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.PID;


@Config
public class DriveWheels {
    public static double maxVelocity = 50;
    // BackEMF constants
    public static double Ex = 0.9;
    public static double Ey = 0.7;
    public static double Eh = -2.1;
    public static double Ed = -2.1;

    // Tanh scaling
    public static double tsh = 100;
    public static double tsv = 100;

    // Loopback constants
    public static double Lxk = 0.3;
    public static double Lhk = -0.2;
    public static double Lml = 0.15;
    public static double Lmr = 0.15;
    public static double Lmk = 0.1;




    public static MPCPath.MPCParams defaultParams = new MPCPath.MPCParams();

    public static double driveAcceleration = 900;
    public static double angularAcceleration = 30;


    public static double controlLimit = 0.5;

    public static double LeverArm = 20;
    public static double Kih = 0;
    public static double Kpv = 0;
    public static double Kvh = 0;
    public static double strength = 1;


    /*  Drive wheel Parameterization
    Each wheel is specified as a unitless 3D Vector, characterizing what direction the wheel moves in.
    Heading is included in the vector.
    See https://files.andymark.com/2008CON-Omni-Baker-McKenzie.pdf for details on math.

    Note that the Motor power unit is 1/(Battery level) of a volt. This could be useful for battery correction in the future.

    The trick is to separate u * (v_t + v_r) to u * v_t + u * v_r. Treat v_r as omega * leverarm, and then u * v_r becomes |u| * leverarm * omega.
    This can then be added to u * v_t to get the final dot product. If you then just put it in the vector, and have the third component be the
    angular velocity, it all works because v_r = leverarm * omega.

    For most cases, these will all be -1 or +1 because it's a square drive base with each wheel having equal torque.
    */
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

    public static PID.PIDCoefficients driveConstants = new PID.PIDCoefficients(0.03, 0.01, 0, 0.03, 0.01);

}
