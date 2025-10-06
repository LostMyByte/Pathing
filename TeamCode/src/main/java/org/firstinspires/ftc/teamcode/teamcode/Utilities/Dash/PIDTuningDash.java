package org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDTuningDash {

    //Old stuff since before summer camps, not sure if still needed
//    public static double ITP = -0.28;
//    public static double ITD = 0;
    public static double LinearAngleConstant = 0.075;
    public static double SinusoidalAngleConstant = 0;

    public static double EP = 0;
    public static double EI = 0;
    public static double ED = 0;

    public static double AP = 0;
    public static double AI = 0;
    public static double AD = 0;
    public static double AFF = 0;
    public static double HP = -0.7;
    public static double HD = 0;
    public static double rateOfChange = 1;
    //New stuff, this definitely needs to be looked at

    public static double DeadZone = 0;



    public static  double Kph = 0.03;
    public static  double Kdh = -0.000;
    public static double Kih = 0;

    public static double ShooterP = 0.003;
    public static double ShooterD = 0;
    public static double ShooterF = 0.0;

    public static double VSlidesP = 0.004   ;
    public static double VSlidesD = 0.004;
    public static double VSlidesF = 0;

    public static double VSlidesPCLIMB = 0.008;
    public static double VSlidesDCLIMB = 0.004;
    public static  double Kpd = -0.05;
    public static  double Kdd = 0.005;
    public static double Kfd = -0.1;

    public static double kVd = 0.012;
    public static double maxVd = 70;
    public static double acceld = 100;
    public static  double Kps = -0.07;
    public static  double Kds = 0;
    public static double Kfs = -0.1;

    public static double deceld = 80;
    public static double deceldClose = 10;
    public static double Kld = 0.015;
    public static double velPd = 0.02;

    public static double kVs = 0.025;
    public static double maxVs = 60;
    public static double accels = 60;
    public static double decels = 40;
    public static double decelsClose = 10;
    public static double Kls = 0.035;
    public static double velPs = 0.03;

}
