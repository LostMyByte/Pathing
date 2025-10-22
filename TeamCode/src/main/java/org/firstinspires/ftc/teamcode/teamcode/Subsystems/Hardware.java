package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

//hardware variables followed by hardware objects

//DCMotorEx: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/DcMotorEx.html

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.MecanumDrive;
@Config
public class Hardware{
    
    
    public static final String tagCam = "TagCam";
    public static final String sparkFun = "OTOS";
    public static final Size cameraResolution = new Size(1280,720);
    public static String
    extendo = "chub2gray", boxTubeL = "motorL", boxTubeR = "motorR";

    public static final String
   //         verticalEncoder = "bl", horizontalEncoder = "fr";
    //V2:
    verticalEncoder = "fr", horizontalEncoder = "br";

    public static final String
            depositorServo = "depositor";
    //                                        ehub 5
    public static final String
            transfer = "transfer";

    public static final String
            indicatorLight = "ehub1none", angleWrapWarningLight = "angleWrapWarningLight",shooter1 = "left", shooter2 = "right";

    public static final String
            grabLeft = "grabL", grabRight = "grabR";
    public static final String
            v4b1 = "Sv4bL", v4b2 = "Sv4bR", wrist = "SWrist";

    public static final String
            clawLeft = "clawL", clawRight = null; // Trust

    public static final  String
            v4bEncoder = "v4bSensor", wristEncoder =  "wristSensor", hood = "hood", turret = "turret";

    public static final String
            climb1 = "launcher", climb2 = "climb";


    public static final String
            differentialLeft = "ehub3brown", differentialRight = "chub1green", intakeArmRight = "shub4purple", intakeLeft = "shub1black", intakeRight = "shub3white", intakeArmLeft = "shub2gray", deposotorArmLeft = "ehub5orange", bottomSweeper = "ehub0",  depositorArmRight = "chub5blue", depositorClaw = "ehub4yellow";

    public static final String odoWheels = "odoWheels";

    //Multipliers for Test Chassis
    //public static double verticalEncoderTicksToCM = -0.00075491,   horizontalEncoderTicksToCM = -0.00075227449;

    //Multipliers for Jamie V2
    public static double verticalEncoderTicksToCM = -0.00091912743, horizontalEncoderTicksToCM = -0.0005236551576;


    public static final String
            leftFront = "chub1orange", rightFront  = "ehub1blue",
            leftBack = "chub0white", rightBack = "ehub0red";



    public static final double[] mecanumWheelPowerVector = new double[]{MecanumDrive.MecanumDriveDash.vecX,MecanumDrive.MecanumDriveDash.vecY};
    public static String limelight = "limelight";

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

