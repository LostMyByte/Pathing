package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities;

import static java.lang.Math.abs;
import static java.lang.Math.floorMod;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Comparator;

public class MathUtils {

    public static double deadZone(double number, double deadZone){

        return number < deadZone && number > -deadZone ? 0 : number;

    }

    public static double sigmoid(double rawNumber, double range, double outputRange){
        return (outputRange / (1 + Math.pow(range, -rawNumber))) - outputRange / 2;
    }

    public static double degsToRads(Double theta){
        return theta * (Math.PI / 180);
    }
    
    public static double radsToDegs(Double theta){ return theta * (180 / Math.PI); }
    
    
    public static double degSin(double theta){ return Math.sin(degsToRads(theta)); }
    
    public static double degCos(double theta){ return Math.cos(degsToRads(theta)); }
    
    public static double degTan(double theta){ return Math.tan(degsToRads(theta)); }
    
    public static double degASin(double sin){ return radsToDegs(Math.asin(sin)); }
    
    public static double degATan(double opposite, double adjacent){ return radsToDegs(Math.atan2(opposite, adjacent)); }


    public static double mod(double value, int base){
        return (value % base + base) % base;
    }
    
    public static double pow(double value, double exponent) {
        if(value == 0) return 0;
        else return Math.pow(abs(value), exponent) * (value / abs(value));
    }
    public static double absMax(double value1, double value2){
        return Math.max(abs(value1), abs(value2));
    }

    //TODO  I need to do a lot more documentation on my code as I write it -  so I can do a better job of maintenance
    //TODO Make these actually have input for accuracy like a normal function
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(double dividend, double divisor){
        return floorMod(Math.round(dividend * 1e6), Math.round(divisor * 1e6)) / 1e6;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(double dividend, int divisor){
        return Math.floorMod(Math.round(dividend * 1e6), divisor) / 1e6;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(int dividend, double divisor){
        return floorMod(dividend, Math.round(divisor * 1e6)) / 1e6;

    }

    //calculates distance between two points
    public static double distance2D(double x1, double y1, double x2, double y2){
        return abs(Math.hypot(x2-x1, y2-y1));
    }


    public static double angleFromOrigin(double X, double Y){
        return ((270 - (Math.atan2(0 - Y, 0 - X)) * 180 / Math.PI) % 360);
    }

    /**
     * x1 and y1 should be for the current location, x2 and y2 for the target location
     * THIS IS FOR THE AUTO AND WONT WORK WITH TRADITIONAL COORDINATE SYSTEMS
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    public static double angleBetweenPoints(double x1, double y1, double x2, double y2){
        double xShifted = x2 - x1;
        double yShifted = y2 - y1;
        return angleFromOrigin(xShifted, yShifted) + 180;
    }

    /*public static Point shift(Point p, double shiftAngle){
        double rawX = p.x;
        double rawY = p.y;
        double x = (rawX * Math.cos(Math.toRadians(shiftAngle))) - (rawY * Math.sin(Math.toRadians(shiftAngle)));
        double y = (rawX * Math.sin(Math.toRadians(shiftAngle))) + (rawY * Math.cos(Math.toRadians(shiftAngle)));
        return new Point(x, y);
    }*/

    public static boolean withinRange(double value, double min, double max){
        return (value >= min) && (value <= max);
    }

    public static double linearConversion(double value, double oldMin, double oldMax, double newMin, double newMax){
        double oldRange = (oldMax - oldMin);
        double newRange = (newMax - newMin);
        return (((value - oldMin) * newRange) / oldRange) + newMin;
    }

    public static double middle(double num1, double num2){
        double avg = (num1 + num2) / 2;
        return avg;
    }

    public static double median(ArrayList<Double> numArray){
        numArray.sort(Comparator.naturalOrder());
        double median;
        if(numArray.size() % 2 == 0){
            median = (numArray.get(numArray.size() / 2) + numArray.get(numArray.size() / 2 - 1)/ 2);
        }else{
            median = numArray.get(numArray.size() / 2);
        }
        return median;
    }

    public static double closestTarget(double targetAngle, double currentAngle, double mod){
        double simpleTargetDelta = Math.floorMod(Math.round(((mod - targetAngle) + currentAngle) * 1e6), Math.round((mod + 0.000) * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (mod - simpleTargetDelta);
        return Math.abs(simpleTargetDelta) <= Math.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
    }

    public static Point subtractPoints(Point minuend, Point subtrahend){
        return new Point(minuend.x - subtrahend.x, minuend.y - subtrahend.y);
    }

    public static Point addPoints(Point point1, Point point2){
        return new Point(point1.x + point2.x, point1.y + point2.y);
    }

    public static Point multiplyPoint(Point point, double multiplier){
        return new Point(point.x * multiplier, point.y * multiplier);
    }

    public static Double distBetweenPoints(Point p1, Point p2){
        return Math.hypot(p1.x -  p2.x, p1.y - p2.y);
    }

    public static Point midPoint(Point p1, Point p2){
        return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    }

    public static Vector2d addVector2d(Vector2d vector1, Vector2d vector2){
        return new Vector2d(vector1.getX() + vector1.getX(), vector1.getY() + vector2.getY());
    }

    /**
     * for when you want to have like a PID that can't overpower a driver control,
     * authority of secondary is inversely proportional to magnitude of priority
     * @param priority
     * @param secondary
     * @return
     */
    public static double combinePriority(double priority, double secondary){
        double authority = -abs(priority) + 1;
        return priority + (secondary * authority);
    }

    /**
     * Interpolates number n on range start1->end1 to a number in the second range, linearly
     * @param n
     * @param start1
     * @param end1
     * @param start2
     * @param end2
     * @return
     */
    public static double interpolateRanges(double n, double start1, double end1, double start2, double end2){
        double ratio = (n - start1) / (end1 - start1);
        return start2 + (ratio * (end2 - start2));
    }

    public static double characterizeResponse(double n, double upper, double lower, double deadzone){
        if(n < -deadzone / 2) {
            return n + (deadzone / 2) + lower;
        }else if(n > deadzone / 2){
            return n - (deadzone / 2) + upper;
        }else{
            return ((upper - lower) / deadzone) * (n - deadzone / 2) + upper;
        }
    }

    /**
     * https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
     * @param radius
     * @param p1
     * @param p2
     * @return
     */
    public static Point getCircleIntercept(double radius, Point p1, Point p2){
        Vector2d d = new Vector2d(subtractPoints(p2, p1));
        Vector2d f = new Vector2d(p1);

        double a = d.dot(d);
        double b = 2.0 * f.dot(d);
        double c = f.dot(f) - radius * radius;

        double discriminant = (b * b) - (4 * (a * c));
        if(discriminant < 0){
            return p1; //TODO kinda bad exception handling, suboptimal for sure
        }else{
            discriminant = sqrt(discriminant);

            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);

            if(t1 >= 0 && t1 <= 1){
                return addPoints(p1, multiplyPoint(d.asPoint(), t1));
            }else if(t2 >= 0 && t2 <= 1){
                return addPoints(p1, multiplyPoint(d.asPoint(), t2));
            }else{
                return p1;
            }
        }
    }


    /**
     * It's square root, but it works on negative numbers. Mathematically valid?? Optimized? Sane???
     * All these problems and more go away with Wolfpack Machina's patent pending
     * expandedSqrt() (tm) mathUtil function
     * @param n
     * @return
     */
    public static double expandedSqrt(double n){
        return sqrt(abs(n)) * signum(n);
    }


}
