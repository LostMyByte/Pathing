package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

//hardware variables followed by hardware objects

//DCMotorEx: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/DcMotorEx.html

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.MecanumDrive;
@Config
public class Hardware{
    
    
    public static final String limelight = "limelight";
    public static final String sparkFun = "OTOS";
    public static final Size cameraResolution = new Size(1280,720);
    public static final String
            shooter1 = "shooter1",
            shooter2 = "shooter2";


    public static final String
            turret = "turret";

    public static final String
            hood = "hood";


    public static final String
          stiltsFront = "stiltsFront", sliltsBack = "stiltsBack";

    public static final String odoWheels = "odoWheels";

    //Multipliers for Test Chassis
    //public static double verticalEncoderTicksToCM = -0.00075491,   horizontalEncoderTicksToCM = -0.00075227449;

    //Multipliers for Jamie V2
    public static double verticalEncoderTicksToCM = -0.00091912743, horizontalEncoderTicksToCM = -0.0005236551576;


    public static final String
            leftFront = "chub1orange", rightFront  = "ehub1blue",
            leftBack = "chub0white", rightBack = "ehub0red";



    public static final double[] mecanumWheelPowerVector = new double[]{MecanumDrive.MecanumDriveDash.vecX,MecanumDrive.MecanumDriveDash.vecY};


    public static String VslideL = "ehub3green";
    public static String VslideR = "ehub2purple";

    public static String depositor = "depositor";
    public static String specimenClaw = "claw";
    public static String blockCam = "BlockCam";
    public static String depositorDoor = "depositorDoor";
}

//EHUB
//grab
//brown is left - 1
//orange is on right - 3


//v4b
//white is left - 0
//red is right - 4

