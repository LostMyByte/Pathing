package org.firstinspires.ftc.teamcode.Utilities.Configuration;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Motion.Controllers.PID;
import org.firstinspires.ftc.teamcode.Motion.Controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.DriveWheel;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;


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

        public static class FL {
            public static double x = -1.25;
            public static double y = -1;
            public static double h = -1;

        }
        public static class FR {
            public static double x = -1.25;
            public static double y = 1;
            public static double h = -1;

        }
        public static class BL {
            public static double x = 1.25;
            public static double y = -1;
            public static double h = -1;

        }
        public static class BR {
            public static double x = 1.25;
            public static double y = 1;
            public static double h = -1;

        }
        public static PIDCoefficients driveConstants = new PIDCoefficients(0.03, 0.01, 0, 0.03, 0.01);
        public static double driveAcceleration = 20;
    }





    // The drivewheels put into an array so the FixedDriveTrain class can access them easily.
    public static DriveWheel[] driveWheels= new DriveWheel[]{
            new DriveWheel(Hardware.rightFront, new Vector(DriveWheels.FR.x, DriveWheels.FR.y, DriveWheels.FR.h)),
            new DriveWheel(Hardware.leftFront,  new Vector(DriveWheels.FL.x, DriveWheels.FL.y, DriveWheels.FL.h)),
            new DriveWheel(Hardware.rightBack,  new Vector(DriveWheels.BR.x, DriveWheels.BR.y, DriveWheels.BR.h)),
            new DriveWheel(Hardware.leftBack,   new Vector(DriveWheels.BL.x, DriveWheels.BL.y, DriveWheels.BL.h))};



}
