/*
package org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.Team.BLUE;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.Team.RED;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.team;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.minAreaRect;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class IntoTheDeepVisionProcessor implements VisionProcessor, CameraStreamSource {

    private static final String TAG = "IntoTheDeepVisionProcessor";

    public static int max_H_Blue = 120;
    public static int max_S_Blue = 255;
    public static int max_V_Blue = 255;
    // public String team;
    public static Rect largestRect;
    public static RotatedRect largestRotatedRect;
    public static RotatedRect largestRotatedRectBlue;
    public static RotatedRect largestRotatedRectRed;
    public static RotatedRect largestRotatedRectYellow;
    public static int min_H_Blue = 80;
    public static int min_S_Blue = 100;
    public static int min_V_Blue = 50;

    public static int max_H_Red = 150;
    public static int max_S_Red = 255;
    public static int max_V_Red = 255;
    public static int min_H_Red = 110;
    public static int min_S_Red = 140;
    public static int min_V_Red = 70;
    public static int max_H_Yellow = 105;
    public static int max_S_Yellow = 255;
    public static int max_V_Yellow = 255;
    public static int min_H_Yellow = 85;
    public static int min_S_Yellow = 150;
    public static int min_V_Yellow = 100;
    public static double offset = 0;
    public static int erodeConstant = 5;
    public static int dilateConstant = 5;
    // For camera offset, going to try and make it better
    public static int upperAreaDetectionLimit = 4;
  static int lowerAreaDetectionLimit = 50;
    public static boolean targetDetected = false;
    ArrayList<MatOfPoint> contoursBlue = new ArrayList<>();
    ArrayList<MatOfPoint> contoursRed = new ArrayList<>();
    ArrayList<MatOfPoint> contoursYellow = new ArrayList<>();
    List<RotatedRect> rotatedRects = Collections.synchronizedList(new ArrayList<>());
    List<RotatedRect> rotatedRectsTemp = new ArrayList<>();
    private static int IMG_HEIGHT = 0;
    private static int IMG_WIDTH = 0;
    // Sets up variables to collect image details
    public static double[] centerPixelColorRGB = {0, 0, 0};
    public static double[] centerPixelColorBGR = {0, 0, 0};
    private final Scalar lightBlue = new Scalar(3, 252, 227);
    private Mat output = new Mat(),
            maskBlue = new Mat(),
            maskRed = new Mat(),
            maskYellow = new Mat();

    private Mat hierarchy = new Mat();

    // Stuff for variables
   int submatleft = IMG_WIDTH/3;
    int submatright = IMG_WIDTH-IMG_WIDTH/3;
    double widthOfSample = 1.5;
    double focallengthPX = 396.874;
    double cx = 626.13,
            cy = 343.82;
    double focallengthMM = 0; //3.67;
    // Same as fx and fy (focal length on x direction, focal length on y direction)

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    // This is for camera stream to dashboard
    // Can be ignored entirely

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) { // This thing will do the actual stuff
        input.copyTo(output);
        // Clear previous contours
        contoursBlue.clear();
        contoursRed.clear();
        contoursYellow.clear();

        // Synchronize access to rotatedRects
        synchronized (rotatedRects) {
            rotatedRects.clear();
        }

        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        Scalar MIN_THRESH_BLUE = new Scalar(min_H_Blue, min_S_Blue, min_V_Blue);
        Scalar MAX_THRESH_BLUE = new Scalar(max_H_Blue, max_S_Blue, max_V_Blue);
        Scalar MIN_THRESH_RED = new Scalar(min_H_Red, min_S_Red, min_V_Red);
        Scalar MAX_THRESH_RED = new Scalar(max_H_Red, max_S_Red, max_V_Red);
        Scalar MIN_THRESH_YELLOW = new Scalar(min_H_Yellow, min_S_Yellow, min_V_Yellow);
        Scalar MAX_THRESH_YELLOW = new Scalar(max_H_Yellow, max_S_Yellow, max_V_Yellow);
        // Setting up all the color thresholds

        Imgproc.cvtColor(input, maskBlue, COLOR_RGB2HSV);
        Imgproc.cvtColor(input, maskRed, COLOR_BGR2HSV);
        Imgproc.cvtColor(input, maskYellow, COLOR_BGR2HSV);
        // Goes from RGB to HSV color space

        inRange(maskBlue, MIN_THRESH_BLUE, MAX_THRESH_BLUE, maskBlue);
        inRange(maskRed, MIN_THRESH_RED, MAX_THRESH_RED, maskRed);
        inRange(maskYellow, MIN_THRESH_YELLOW, MAX_THRESH_YELLOW, maskYellow);
//slides can go 17 inches

        Rect submatRect = new Rect(new Point(4, 4), new Point(IMG_WIDTH, IMG_HEIGHT));
        maskBlue = maskBlue.submat(submatRect);
        maskRed = maskRed.submat(submatRect);
        maskYellow = maskYellow.submat(submatRect);
        // Actual threshold thing to correct for top of screen being weird and glitchy, can use to only detect part of screen

        erode(maskBlue, maskBlue, new Mat(erodeConstant, erodeConstant, CV_8U));
        dilate(maskBlue, maskBlue, new Mat(dilateConstant, dilateConstant, CV_8U));
        erode(maskRed, maskRed, new Mat(erodeConstant, erodeConstant, CV_8U));
        dilate(maskRed, maskRed, new Mat(dilateConstant, dilateConstant, CV_8U));
        erode(maskYellow, maskYellow, new Mat(erodeConstant, erodeConstant, CV_8U));
        dilate(maskYellow, maskYellow, new Mat(dilateConstant, dilateConstant, CV_8U));
        // This is the weird erode dilate thing that gets rid of stray pixels

        findContours(maskBlue, contoursBlue, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(maskRed, contoursRed, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(maskYellow, contoursYellow, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        largestRotatedRectBlue = getObjectsDetected(contoursBlue);
        largestRotatedRectRed = getObjectsDetected(contoursRed);
        largestRotatedRectYellow = getObjectsDetected(contoursYellow);

        // Synchronize access when modifying rotatedRects
        synchronized (rotatedRects) {
            if (Constants.team == BLUE && largestRotatedRectBlue != null) {
                if (IMG_WIDTH/3<largestRotatedRectBlue.center.x&&largestRotatedRectBlue.center.x < IMG_WIDTH*2/3&&largestRotatedRectBlue.center.y>upperAreaDetectionLimit&&largestRotatedRectBlue.center.y>lowerAreaDetectionLimit){
                    rotatedRects.add(largestRotatedRectBlue);
                    targetDetected=true;
            }}
            if (Constants.team == RED && largestRotatedRectRed != null) {
                if (IMG_WIDTH/3<largestRotatedRectRed.center.x&&largestRotatedRectRed.center.x < IMG_WIDTH*2/3&&largestRotatedRectRed.center.y>upperAreaDetectionLimit&&largestRotatedRectRed.center.y>lowerAreaDetectionLimit){
                    rotatedRects.add(largestRotatedRectRed);
                    targetDetected=true;
            }}
            if (largestRotatedRectYellow != null) {
                if (IMG_WIDTH/3<largestRotatedRectYellow.center.x&&largestRotatedRectYellow.center.x < IMG_WIDTH*2/3&&largestRotatedRectYellow.center.y>upperAreaDetectionLimit&&largestRotatedRectYellow.center.y>lowerAreaDetectionLimit){
                    rotatedRects.add(largestRotatedRectYellow);
                    targetDetected=true;
                }}

            if (!rotatedRects.isEmpty() && rotatedRects.get(0) != null) {
                largestRotatedRect = VisionUtils.sortRotatedRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rotatedRects).get(0);
            } else {
                largestRotatedRect = null; // Ensure it's set to null if no rectangles are found
            }

            if (rotatedRects.isEmpty()) {
                Log.d(TAG, "processFrame: No rotated rectangles detected.");
            } else {
                Log.d(TAG, "processFrame: Detected rotatedRects size: " + rotatedRects.size());
            }
        }



        // Figures out all the pixels on the edges of the blob, useful for finding center

        Bitmap b = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(output, b);
        lastFrame.set(b);
        // This is used to determine which Mat is output to dashboard

        return output;
    }

    public double[] getDisplacement(boolean lookForYellow) {

     //   synchronized (rotatedRects) {
            if (lookForYellow) {
                // If looking for yellow, ensure largestRotatedRectYellow is detected
                if (largestRotatedRectYellow == null) {
                    Log.d(TAG, "getDisplacement: largestRotatedRectYellow is null while looking for yellow.");
                    return null;
                }
            } else {
                // If not looking for yellow, ensure rotatedRects has at least one non-null element
                //if (rotatedRects.isEmpty() || rotatedRects.get(0) == null) {
                   // Log.d(TAG, "getDisplacement: rotatedRects is empty or first element is null while not looking for yellow.");
                    //return null;
              //  }
            }

            double[] dimensions = new double[3];
            Point[] points = new Point[4];

            if (lookForYellow && largestRotatedRectYellow!= null) {
                largestRotatedRectYellow.points(points);
            } else if(team == RED && largestRotatedRectRed!=null){
                largestRotatedRectRed.points(points);
            } else  if (team == BLUE && largestRotatedRectBlue!=null){
                largestRotatedRectBlue.points(points);
            }else{
                return null;
            }

            Point choice = points[0];
            for (int i = 1; i < 4; i++) {
                dimensions[i - 1] = Math.sqrt(Math.pow(points[i].x - choice.x, 2) + Math.pow(points[i].y - choice.y, 2));
            }

            Arrays.sort(dimensions);

            double width = dimensions[0];
            double length = dimensions[1];
            double crossSectionalLength = dimensions[2];
            double x = 0;
            double y = 0;
            double h = (widthOfSample * focallengthPX)/ (width);//+widthOfSample/2;
            double z = h - (focallengthMM / DistanceUnit.mmPerInch) ;
            if (!targetDetected) return null;
          if (team == BLUE && largestRotatedRectBlue!=null){
             x = ((largestRotatedRectBlue.center.x - cx) * h) / focallengthPX;
             y = ((largestRotatedRectBlue.center.y- cy ) * h) / focallengthPX;
          } else  if (team == RED&& largestRotatedRectRed!=null){
             x = ((largestRotatedRectRed.center.x - cx) * h) / focallengthPX;
             y = ((largestRotatedRectRed.center.y- cy ) * h) / focallengthPX;
         }

          if (lookForYellow && largestRotatedRectYellow!= null) {
              x = ((largestRotatedRectYellow.center.x - cx) * h) / focallengthPX;
              y = ((largestRotatedRectYellow.center.y - cy) * h) / focallengthPX;
          }




        if(!Double.isNaN(x) && !Double.isNaN(y)&& !Double.isNaN(z)){
            BaseOpMode.addData("xOriginal", x);
            BaseOpMode.addData("yOriginal", y);
            BaseOpMode.addData("zOriginal", z);
            double[] goodDisplacement = new double[2];

            double zprime = z * Math.cos(Math.toRadians(MainIntake.CameraMount.pitchAngle))-y * Math.sin(Math.toRadians(MainIntake.CameraMount.pitchAngle)); // Angle of the intake
            if (zprime < 30) {
                goodDisplacement[0] = x * Math.cos(Math.toRadians(MainIntake.CameraMount.yawAnlge)) - zprime * Math.sin(Math.toRadians(MainIntake.CameraMount.yawAnlge));
                goodDisplacement[1] = x * Math.sin(Math.toRadians(MainIntake.CameraMount.yawAnlge)) + zprime * Math.cos(Math.toRadians(MainIntake.CameraMount.yawAnlge));
                //Camera Offsets
                goodDisplacement[0] += MainIntake.CameraMount.xOffset;
                goodDisplacement[1] += MainIntake.CameraMount.yOffset;
            }
            BaseOpMode.addData("zprime", zprime);
            BaseOpMode.addData("xnew", goodDisplacement[0]);
            BaseOpMode.addData("ynew", goodDisplacement[1]);

            //  ready = true;

            return goodDisplacement;

       }
        return null;
    }

    public static double getError() {
        if (targetDetected) {
            double centerRect = largestRect.x + (largestRect.width / 2);
            // Error 157
            return (IMG_WIDTH / 2) - centerRect;
        } else {
            return 0;
        }
    }

    public static double getOffset(){
        if (targetDetected ){
            // Implement offset logic if needed
        }
        return 0;
    }

    public double getCenterPixelRGB2HSV(int arrayIndex){
        return centerPixelColorRGB[arrayIndex];
    }

    public double getCenterPixelBGR2HSV(int arrayIndex){
        return centerPixelColorBGR[arrayIndex];
    }

    public double getCenter() {
        double centerRect = 0;
        if (largestRect != null) {
            centerRect = largestRect.x + (largestRect.width / 2) + offset;
        }
        return centerRect;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // This code runs when you set up the camera processor
        // I mostly just leave this bit alone

        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        // More dashboard setup
    }

    public double getAngleYellow() {
        if (largestRotatedRectYellow != null) {
            double angle = largestRotatedRectYellow.angle;
            if (largestRotatedRectYellow.size.height > largestRotatedRectYellow.size.width) {
                    angle = angle - 90;
            }
            while (angle > 90) {
                angle = angle - 180;
            }
            while (angle < -90) {
                angle = angle + 180;
            }

            return angle;
        } else {
            return 0;
        }
    }

    public double getAngle() {

        double angle;
       // synchronized (rotatedRects) {
          //  if (!rotatedRects.isEmpty() && rotatedRects.get(0) != null) {
                //double angle = rotatedRects.get(0).angle;
              //  if (rotatedRects.get(0).size.height > rotatedRects.get(0).size.width) {
            if (team == BLUE && largestRotatedRectBlue!= null){
                angle = largestRotatedRectBlue.angle;
                if( largestRotatedRectBlue.size.height>largestRotatedRectBlue.size.width){
                    angle = angle - 90;
                }
                while (angle > 90) {
                    angle = angle - 180;
                }
                while (angle < -90) {
                    angle = angle + 180;
                }

                return angle;
            }

            else if (team == RED && largestRotatedRectRed!= null){
             angle = largestRotatedRectRed.angle;
            if( largestRotatedRectRed.size.height>largestRotatedRectRed.size.width){
                angle = angle - 90;
            }
            while (angle > 90) {
                angle = angle - 180;
            }
            while (angle < -90) {
                angle = angle + 180;
            }

            return angle;

        } else {
            return 0;
        }


    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
      //  Rect submatRect = new Rect(new Point(IMG_WIDTH/3, upperAreaDetectionLimit), new Point(IMG_WIDTH-IMG_WIDTH/3, IMG_HEIGHT));
        Rect submatRect = new Rect(new Point(IMG_WIDTH/3, upperAreaDetectionLimit), new Point(IMG_WIDTH-IMG_WIDTH/3, IMG_HEIGHT));

        Paint rectPaintYellow = new Paint();
        rectPaintYellow.setColor(Color.YELLOW);
        rectPaintYellow.setStyle(Paint.Style.STROKE);
        rectPaintYellow.setStrokeWidth(scaleCanvasDensity * 8);

        Paint rectPaintRed = new Paint();
        rectPaintRed.setColor(Color.RED);
        rectPaintRed.setStyle(Paint.Style.STROKE);
        rectPaintRed.setStrokeWidth(scaleCanvasDensity * 8);

        Paint rectPaintBlue = new Paint();
        rectPaintBlue.setColor(Color.CYAN);
        rectPaintBlue.setStyle(Paint.Style.STROKE);
        rectPaintBlue.setStrokeWidth(scaleCanvasDensity * 8);

        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 8);

        Paint contourpaintBlue = new Paint();
        contourpaintBlue.setColor(Color.CYAN);
        contourpaintBlue.setStyle(Paint.Style.STROKE);
        contourpaintBlue.setStrokeWidth(scaleCanvasDensity *4);

        Paint contourpaintYellow = new Paint();
        contourpaintYellow.setColor(Color.YELLOW);
        contourpaintYellow.setStyle(Paint.Style.STROKE);
        contourpaintYellow.setStrokeWidth(scaleCanvasDensity *4);

        Paint contourpaintRed = new Paint();
        contourpaintRed.setColor(Color.RED);
        contourpaintRed.setStyle(Paint.Style.STROKE);
        contourpaintRed.setStrokeWidth(scaleCanvasDensity *4);

        canvas.drawRect(makeGraphicsRect(submatRect, scaleBmpPxToCanvasPx),rectPaint);
        if(targetDetected) {
            if (largestRect != null) {
                // canvas.drawRect(makeGraphicsRect(largestRect, scaleBmpPxToCanvasPx), rectPaint);
            }

            // Draw the largest rotated rectangle
            if (largestRotatedRectRed != null) {
                // Get the points from the rotated rectangle
                Point[] rectPoints = new Point[4];
                largestRotatedRectRed.points(rectPoints);

                // Draw lines between consecutive points
                for (int i = 0; i < 4; i++) {
                    Point pt1 = rectPoints[i];
                    Point pt2 = rectPoints[(i + 1) % 4]; // Next point, wrapping around
                    canvas.drawLine((float) (pt1.x * scaleBmpPxToCanvasPx),
                            (float) (pt1.y * scaleBmpPxToCanvasPx),
                            (float) (pt2.x * scaleBmpPxToCanvasPx),
                            (float) (pt2.y * scaleBmpPxToCanvasPx),
                            rectPaintRed);
                }
            }
            if (largestRotatedRectBlue != null) {
                // Get the points from the rotated rectangle
                Point[] rectPoints = new Point[4];
                largestRotatedRectBlue.points(rectPoints);

                // Draw lines between consecutive points
                for (int i = 0; i < 4; i++) {
                    Point pt1 = rectPoints[i];
                    Point pt2 = rectPoints[(i + 1) % 4]; // Next point, wrapping around
                    canvas.drawLine((float) (pt1.x * scaleBmpPxToCanvasPx),
                            (float) (pt1.y * scaleBmpPxToCanvasPx),
                            (float) (pt2.x * scaleBmpPxToCanvasPx),
                            (float) (pt2.y * scaleBmpPxToCanvasPx),
                            rectPaintBlue);
                }
            }
            if (largestRotatedRectYellow != null) {
                // Get the points from the rotated rectangle
                Point[] rectPoints = new Point[4];
                largestRotatedRectYellow.points(rectPoints);

                // Draw lines between consecutive points
                for (int i = 0; i < 4; i++) {
                    Point pt1 = rectPoints[i];
                    Point pt2 = rectPoints[(i + 1) % 4]; // Next point, wrapping around
                    canvas.drawLine((float) (pt1.x * scaleBmpPxToCanvasPx),
                            (float) (pt1.y * scaleBmpPxToCanvasPx),
                            (float) (pt2.x * scaleBmpPxToCanvasPx),
                            (float) (pt2.y * scaleBmpPxToCanvasPx),
                            rectPaintYellow);
                }
            }
            if (largestRotatedRect != null) {
                // Get the points from the rotated rectangle
                Point[] rectPoints = new Point[4];
                largestRotatedRect.points(rectPoints);

                // Draw lines between consecutive points
                for (int i = 0; i < 4; i++) {
                    Point pt1 = rectPoints[i];
                    Point pt2 = rectPoints[(i + 1) % 4]; // Next point, wrapping around
                    canvas.drawLine((float) (pt1.x * scaleBmpPxToCanvasPx),
                            (float) (pt1.y * scaleBmpPxToCanvasPx),
                            (float) (pt2.x * scaleBmpPxToCanvasPx),
                            (float) (pt2.y * scaleBmpPxToCanvasPx),
                            rectPaint);
                }
            }

            // Create a copy of the contours
            List<MatOfPoint> contourCopyBlue = new ArrayList<>(contoursBlue);
            // Rectangle showing camera view
            // This loops through all the contours and draw points on the canvas

            for (MatOfPoint point : contourCopyBlue) {
                Point[] contourArray = point.toArray();
                // This Extracts the contour points and iterates through them.
                for (Point p : contourArray) {
                    canvas.drawPoint((float) (p.x * scaleBmpPxToCanvasPx), (float) (p.y * scaleBmpPxToCanvasPx), contourpaintBlue);
                }
            }

            List<MatOfPoint> contourCopyRed = new ArrayList<>(contoursRed);
            // Rectangle showing camera view
            // This loops through all the contours and draw points on the canvas

            for (MatOfPoint point : contourCopyRed) {
                Point[] contourArray = point.toArray();
                // This Extracts the contour points and iterates through them.
                for (Point p : contourArray) {
                    canvas.drawPoint((float) (p.x * scaleBmpPxToCanvasPx), (float) (p.y * scaleBmpPxToCanvasPx), contourpaintRed);
                }
            }

            List<MatOfPoint> contourCopyYellow = new ArrayList<>(contoursYellow);
            // Rectangle showing camera view
            // This loops through all the contours and draw points on the canvas

            for (MatOfPoint point : contourCopyYellow) {
                Point[] contourArray = point.toArray();
                // This Extracts the contour points and iterates through them.
                for (Point p : contourArray) {
                    canvas.drawPoint((float) (p.x * scaleBmpPxToCanvasPx), (float) (p.y * scaleBmpPxToCanvasPx), contourpaintYellow);
                }
            }

        }}

        private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
            int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

            return new android.graphics.Rect(left, top, right, bottom);
        }
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        public RotatedRect getObjectsDetected(ArrayList<MatOfPoint> contours){
            rotatedRectsTemp.clear();


            List<Rect> rects = new ArrayList<>();
            for (int i = 0; i < contours.size(); i++) {
                Rect rect = boundingRect(contours.get(i));
                rects.add(rect);
            } // Creates a bounding rectangle


            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
                RotatedRect rotatedRect = minAreaRect(contour2f);
                rotatedRectsTemp.add(rotatedRect);
            } // Creates a bounding rectangle



            if (!rotatedRectsTemp.isEmpty()) {
                this.largestRotatedRect = VisionUtils.sortRotatedRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rotatedRectsTemp).get(0);
            } else {
                this.largestRotatedRect = null;
            }
            if (!rects.isEmpty() ) {
                this.largestRect = VisionUtils.sortRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rects).get(0);
              //  targetDetected = true;
            } else if (!rotatedRects.isEmpty()){
                this.largestRotatedRect = VisionUtils.sortRotatedRectsByMaxOption(1,VisionUtils.RECT_OPTION.AREA, rotatedRects).get(0);
                //targetDetected = true;
            }else{
                largestRect = null;
                targetDetected = false;
            }
            centerPixelColorBGR = maskRed.get(IMG_WIDTH / 2, IMG_HEIGHT / 2);
            centerPixelColorRGB = maskBlue.get(IMG_WIDTH / 2, IMG_HEIGHT / 2);

            drawContours(output, contours, -1, lightBlue);
            // Draws contours around shapes

            return largestRotatedRect;
        }
    }
*/
