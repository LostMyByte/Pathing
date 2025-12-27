// Primary Author: Mixed
package org.firstinspires.ftc.teamcode.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;


public class Servos {


    @Config
    public static class ServosDash {

        public static double depositorArm0 = 0.32;
        public static double depositorArm90 = 0.1;
        public static double intakeV4b90 = 0;
        public static double intakeV4b0 = 0.45;
        public static double grabberOpenLeft = 0.2;
        public static double grabberCloseLeft = 0.1;
        public static double grabberOpenRight = 0.28;
        public static double grabberCloseRight = 0.18;
        public static double v4b0 = 0.5;
        public static double v4b90 = 0.92;
        public static double intakeWrist0 = 0.45;
        public static double intakeWrist90 = 0.1;
        public static double leftClose = 0.22;
        public static double leftOpen = 0.48;
        public static double rightClose = 0.58;
        public static double rightOpen = 0.3;

        public static double leftBetween = 0.34;
        public static double rightBetween = 0.4;
        public static double wrist90 = 0.7;
        public static double wrist0 = 0.5;
        public static double downPos1 = 1;
        public static double downPos2 = 0.15;
        public static double climbPos1 = 0.9;
        public static double climbPos2 = 0.6;
        public static double holdPlane = 0.7;
        public static double launchPlane = 1;
        public static double specimenOpen;
        public static double specimenClose;
        public static double intakeArmLeft0 = 0.76;
        public static double intakeArmLeft90 = 0.37;
        public static double intakeArmRight0 = 0.25;
        public static double intakeArmRight90 = 0.62;

        public static double intakeTurret45 = 0.84;
        public static double intakeTurret0 = 0.55;
    }

    public static double intakeV4bPos = 0;

    public static class Hood extends Servo {
        public Hood() {
            super(Hardware.hood, 0.9, 65, 0.62, 25);
        }

        @Override
        public void setPosition(double p) {
            super.setPosition(p);
        }

        @Override
        public void setPositionInterpolated(double a) {
            super.setPositionInterpolated(a);
        }
    }



    public static class Turret extends Servo{
        public Turret(){super(Hardware.turret,.92,Math.PI/2,.41,-Math.PI/2);}
    }
    public static class Turret2 extends Servo{
        public Turret2(){super(Hardware.turret2,.92,Math.PI/2,.41,-Math.PI/2);}
    }
    public static class FliPrampFront extends Servo{
        public FliPrampFront(){super(Hardware.fliPrampFront);}
        public void shoot(){this.setPosition(.22);}
        public void mid(){this.setPosition(.15);}
        public void lessMid(){this.setPosition(.12);}
        public void flat(){this.setPosition(.08);}
    }
    public static class FliPrampBack extends Servo {
        public FliPrampBack(){super(Hardware.fliPrampRear);}
        public void shoot(){this.setPosition(.7);}
        public void mid(){this.setPosition(.82);}
        public void lessMid(){this.setPosition(.83);}
        public void flat(){this.setPosition(.87);}
    }
}




    //For everything, 0 is straight up, 90 is facing forwards towards the intake

