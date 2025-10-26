// Primary Author: Past Team Members
package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities;

import org.opencv.core.Point;

//@Config
public class VisionDash {

    public static Point target = new Point(0, 0);

    public static boolean debugMode;

    public static int blur = 75;
    public static int erode_const = 6;
    public static int dilate_const = 1;


    public static int max_V = 255;
    public static int max_S = 255;
    public static int max_H = 115;

    public static int min_V = 20;
    public static int min_S = 120;
    public static int min_H = 90;

    public static int blck_max_V = 255;
    public static int blck_max_S = 45;
    public static int blck_max_H = 255;

    public static int blck_min_V = 0;
    public static int blck_min_S = 0;
    public static int blck_min_H = 0;

    public static int blackAreaThresh = 18000 / 76800;
    public static int mainAreaThresh = 260 / 76800;
    public static double mainWidthThresh = 6;
    public static double mainAspectThresh = 1.8;



    public static double[] YCbCrReadout = {0, 0, 0};

}



