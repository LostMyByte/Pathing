package org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public class TankDrivetrain {

    @Config
    public static class TankDriveConfig {
        public static String L1 = "fl";
        public static String L2 = "bl";
        public static String R1 = "fr";
        public static String R2 = "br";

        public static boolean L1Reversed = false;
        public static boolean L2Reversed = false;
        public static boolean R1Reversed = true;
        public static boolean R2Reversed = true;
    }

    Motor L1;
    Motor L2;
    Motor R1;
    Motor R2;

    public TankDrivetrain() {

        this.L1 = new Motor(TankDriveConfig.L1, TankDriveConfig.L1Reversed);
        this.L2 = new Motor(TankDriveConfig.L2, TankDriveConfig.L2Reversed);
        this.R1 = new Motor(TankDriveConfig.R1, TankDriveConfig.R1Reversed);
        this.R2 = new Motor(TankDriveConfig.R2, TankDriveConfig.R2Reversed);
    }

    public void move(double drive, double turn, double speed) {
        double powerLeft = (drive - turn) * speed;
        double powerRight = (drive + turn) * speed;

        L1.setPower(powerLeft);
        L2.setPower(powerLeft);
        R1.setPower(powerRight);
        R2.setPower(powerRight);
    }

    public void veryVeryDirectDrive(double drive, double turn){
        double powerLeft = (drive - turn);
        double powerRight = (drive + turn);

        L1.setPower(powerLeft);
        L2.setPower(powerLeft);
        R1.setPower(powerRight);
        R2.setPower(powerRight);
    }

    public void veryVeryDirectDrive(double drive, double turn, double speed){
        double powerLeft = (drive - turn)*speed;
        double powerRight = (drive + turn)*speed;

        L1.setPower(powerLeft);
        L2.setPower(powerLeft);
        R1.setPower(powerRight);
        R2.setPower(powerRight);
    }
}
