// Primary Author: Mixed
package org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {
    public static double fieldSizeY = 141.5;
    public static double fieldSizeX = 141.5;
    public static double backWallY = 0;

    public static double goalAprilTagHeight = 0.7493; //meters
    public static double limelightLensHeightFromGround = 0.1143; //meters, this is for test setup
    public static double limelightAngleOffset = 22.5;//degrees, this is again for test setup
    public static double visionTurnDeadzone = 5; //silly (ignore this stuff)
    public static double VisionTurn = 0.008;
    public static double VisionDrive = -0.012;
    public static double VisionDriveDeadzone = 1;
    public static double VisionDistanceTarget = 180;
    public static double goalAngle = -53;
    public static double goalAprilTagCornerDistanceX = -42;
    public static double goalAprilTagCornerDistanceY = 27.5;
    public static double LimeLightOffsetRadius = 12.5;
    public static double TurretOffsetX = 7.5;
    public static double TurretOffsetY = 0;
    public double angleRad;
    public static double tag16x = fieldSizeX-22.25;
    public static double tag11x = 23.25;
    public static double tag12y = 70.25;
    public static double tag13x = 23.25;
    public static double tag14x = fieldSizeX - 24.25;
    public static double tag15y = 70.25;


    public static double robotSizeX = 16;
    //The Size of the robot in the direction the wheels are facing
    public static double halfRobotSizeX = robotSizeX / 2;
    public static double robotSizeY = 17.75;
    //The size of the robot in the direction the wheels are not facing
    public static double halfRobotSizeY = robotSizeY / 2;

    public static double tagCamOffsetX;
    public static double tagCamOffsetY;

    public static double startAngle = Double.NaN;
    public static double armLength = 17;
    public static double slidesStartHeightConstant = 16.25;
    public static double specimenClawStartHeightConstant = 0;
    public static double ticksToVSlidesInchesConstant = (38-slidesStartHeightConstant)/1724;
    public static Team team;
    public static Motif motif;
    public static double HSlidesMaxLengthInches = 17.625;
    public static double HSlidesMaxLengthTicks = 727;
    public static double ticksToHSlidesInchesConstant = HSlidesMaxLengthTicks/HSlidesMaxLengthInches;
    //maxes out at 729
    //17.625

    //PID Constants

    public static double HSlidesP, HSlidesI, HslidesD;
    public static double VSlidesP, VSlidesI, VSlidesD, VSlidesF;
    public static double VSlidesDeadzone, VSlidesLowerLimit;


    //For length of slides in inches
    public static double A = 8.887165;
    public static double B = 0.00415547;
    public static double C = 1.50989;
    public static double D = 8.887165;
    public static double[][] obstacles = {{1,22},{2, 3}};
    public static double camOffsetX = 0;
    public static double camOffsetY = 0;
    public static boolean intakeReady = false;
    public static boolean depositorReady = false;
    public static final double fx = 396.874; //updated for global shutter cam
    public static final double cy = 343.82;
    public static final double fy = 396.874;
    public static final double cx = 626.13;

    public static final double focalLengthMM = 30;
    public static double inchesToPixels = fx/(focalLengthMM/DistanceUnit.mmPerInch);
    public static double pixelsToMeters = (focalLengthMM/100/100)/fx;

    public static double lengthOfIntakeIN=6.5;
    //very approximate



    public static double g = 8.8;
    //trust
    //DO NOT USE FOR NON-SHOOTER PURPOSES
/*
    public static double goalAprilTagHeight = 0.7495; //meters
    public static double limelightLensHeightFromGround = 0.28; //meters, this is for test setup
    public static double limelightAngleOffset = 22.5;//degrees, this is again for test setup
*/

    public static double tyFiltered = 0;
    public static double tyAlpha = 0.25; // 0..1, higher = snappier
    public static double biasterm = 0.18;


    public void setEndAngle(double angle){
        startAngle = angle;
    }

    public enum Team {
        RED,
        BLUE,



    }

    public enum Motif{
        PPG,
        PGP,
        GPP,
    }
}
