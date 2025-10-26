// Primary Author: Past Team Members
package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.TrigExtensions.AngleMode.DEGREES;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.util.Range;

public class TrigExtensions {

    public enum AngleMode{
        DEGREES,
        RADIANS
    }

    boolean useDegrees = false;
    boolean clip = true;

    //private HashMap<Double, Double[]> basicCache = new HashMap<Double, Double[]>();

    public TrigExtensions(AngleMode angleMode){
        this.useDegrees = angleMode == DEGREES;
    }

    public void setAngleMode(AngleMode angleMode){
        this.useDegrees = angleMode == DEGREES;
        //recalculate our cached values, but this time, had an idea to recalc but meaningless lol
        //should do this once we cache inverse functs tho
        //Set<Double> cachedVals = basicCache.keySet();
        //basicCache.clear();
        /*
        for (Double value: cachedVals) {
            precalc(value);
        }

         */
    }

    public void precalc(Double... n){
        for (Double value: n) {
            //basicCache.put(value, new Double[] {sin(value), cos(value), tan(value)});
        }
    }

    public void clearCache(){
        //basicCache.clear();
    }

    public double sin(double n){
        return Math.sin(useDegrees ? toRadians(n) : n);
        //basicCache.containsKey(n) ? basicCache.get(n)[0]
        //                :
    }

    public double cos(double n){
        return Math.cos(useDegrees ? toRadians(n) : n);
    }

    public double tan(double n){
        return Math.tan(useDegrees ? toRadians(n) : n);
    }

    public double asin(double n){
        if (clip) n = Range.clip(n, -1.0, 1.0);
        double asin = Math.asin(n);
        return useDegrees ? toDegrees(asin) : asin;
    }

    public double acos(double n){
        if (clip) n = Range.clip(n, -1.0, 1.0);
        double acos = Math.acos(n);
        return useDegrees ? toDegrees(acos) : acos;
    }

    public double atan(double n){
        double atan = Math.atan(n);
        return useDegrees ? toDegrees(atan) : atan;
    }

    public double atan2(double v, double v1){
        double atan2 = Math.atan2(v, v1);
        return useDegrees ? toDegrees(atan2) : atan2;
    }

    /**
     * Defines whether or not to clip function inputs to prevent NaN - true will clip inputs to limited-domain functions
     * @param doClip
     */
    public void setClipping(boolean doClip){
        clip = doClip;
    }

    /*
    public double sec(double n){
        return 1 / Math.cos(useDegrees ? toRadians(n) : n);
    }

    public double csc(double n){
        return 1 / Math.sin(useDegrees ? toRadians(n) : n);
    }

    public double cot(double n){
        return 1 / Math.tan(useDegrees ? toRadians(n) : n);
    }

     */


}
