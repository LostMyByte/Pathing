package org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallDetector.visionDash.maxS_green;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallDetector.visionDash.maxS_purple;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallDetector.visionDash.minV_purple;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.INTER_MAX;
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

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class BallDetector implements VisionProcessor, CameraStreamSource {


    public static Rect largestRect;

    public static Circle largestCircle;
    public static Rect largestPurpleRect;
    public static Rect largestGreenRect;

    public double circleX = 0;
    public double circleY = 0;
    public double circleRadius = 0;
    public static double offset = 0;
    public static int erodeConstant = 5;
    public static int dilateConstant = 5;
    // For camera offset, going to try and make it better
    public static int upperAreaDetectionLimit = 4;
    static int lowerAreaDetectionLimit = 50;
    public static boolean targetDetected = false;
    ArrayList<MatOfPoint> contoursGreen = new ArrayList<>();
    ArrayList<MatOfPoint> contoursPurple = new ArrayList<>();
    private static int IMG_HEIGHT = 0;
    private static int IMG_WIDTH = 0;
    // Sets up variables to collect image details
    public static double[] centerPixelColorRGB = {0, 0, 0};
    public static double[] centerPixelColorBGR = {0, 0, 0};
    private final Scalar lightBlue = new Scalar(3, 252, 227);
    private Mat output = new Mat(),
            maskGreen = new Mat(),
            maskPurple = new Mat();
    private Mat hierarchy = new Mat();

    // Stuff for variables
    int submatleft = IMG_WIDTH/3;
    int submatright = IMG_WIDTH-IMG_WIDTH/3;
    double focallengthPX = 396.874;
    double cx = 626.13,
            cy = 343.82;
    double focallengthMM = 0; //3.67;
    // Same as fx and fy (focal length on x direction, focal length on y direction)

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    // This is for camera stream to dashboard
    // Can be ignored entirely



    @Config
    public static class visionDash{
        public static int maxH_green = 95;
        public static int maxS_green = 255;
        public static int maxV_green = 255;
        public static int minH_green = 75;
        public static int minS_green = 100;
        public static int minV_green = 80;

        public static int maxH_purple = 170;
        public static int maxS_purple = 200;
        public static int maxV_purple = 255;
        public static int minH_purple = 140;
        public static int minS_purple = 55;
        public static int minV_purple = 100;
    }


    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) { //does the actual stuff
        input.copyTo(output);
        // Clear previous contours
        contoursGreen.clear();
        contoursPurple.clear();
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        Scalar MIN_THRESH_GREEN = new Scalar(visionDash.minH_green, visionDash.minS_green, visionDash.minV_green);
        Scalar MAX_THRESH_GREEN = new Scalar(visionDash.maxH_green, maxS_green, visionDash.maxV_green);
        Scalar MIN_THRESH_PURPLE = new Scalar(visionDash.minH_purple, visionDash.minS_purple, minV_purple);
        Scalar MAX_THRESH_PURPLE = new Scalar(visionDash.maxH_purple, maxS_purple, visionDash.maxV_purple);

        Imgproc.cvtColor(input, maskGreen, COLOR_RGB2HSV);
        Imgproc.cvtColor(input, maskPurple, COLOR_RGB2HSV);
        inRange(maskGreen, MIN_THRESH_GREEN, MAX_THRESH_GREEN, maskGreen);
        inRange(maskPurple, MIN_THRESH_PURPLE, MAX_THRESH_PURPLE, maskPurple);

        Rect submatRect = new Rect(new Point(4, 4), new Point(IMG_WIDTH, IMG_HEIGHT));
        maskGreen = maskGreen.submat(submatRect);
        maskPurple = maskPurple.submat(submatRect);
        // Actual threshold thing to correct for top of screen being weird and glitchy, can use to only detect part of screen

        erode(maskGreen, maskGreen, new Mat(erodeConstant, erodeConstant, CV_8U));
        dilate(maskGreen, maskGreen, new Mat(dilateConstant, dilateConstant, CV_8U));
        erode(maskPurple, maskPurple, new Mat(erodeConstant, erodeConstant, CV_8U));
        dilate(maskPurple, maskPurple, new Mat(dilateConstant, dilateConstant, CV_8U));

        findContours(maskGreen, contoursGreen, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(maskPurple, contoursPurple, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        largestPurpleRect = getObjectsDetected(contoursPurple);
        largestGreenRect = getObjectsDetected(contoursGreen);



        Bitmap b = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(output, b);
        lastFrame.set(b);
        // This is used to determine which Mat is output to dashboard

        return output;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        Paint ballPaintGreen = new Paint();
        ballPaintGreen.setColor(Color.GREEN);
        ballPaintGreen.setStyle(Paint.Style.STROKE);
        ballPaintGreen.setStrokeWidth(scaleCanvasDensity * 8);

        Paint contourpaintGreen = new Paint();
        contourpaintGreen.setColor(Color.CYAN);
        contourpaintGreen.setStyle(Paint.Style.STROKE);
        contourpaintGreen.setStrokeWidth(scaleCanvasDensity *4);

        Paint ballPaintPurple = new Paint();
        ballPaintPurple.setColor(Color.RED);
        ballPaintPurple.setStyle(Paint.Style.STROKE);
        ballPaintPurple.setStrokeWidth(scaleCanvasDensity * 8);

        Paint contourpaintPurple = new Paint();
        contourpaintPurple.setColor(Color.MAGENTA);
        contourpaintPurple.setStyle(Paint.Style.STROKE);
        contourpaintPurple.setStrokeWidth(scaleCanvasDensity *4);

        Paint linePaint = new Paint();
        linePaint.setColor(Color.DKGRAY);
        linePaint.setStyle(Paint.Style.STROKE);
        linePaint.setStrokeWidth(scaleCanvasDensity*10);
        // Create a copy of the contours
        List<MatOfPoint> contourCopyPurple = new ArrayList<>(contoursPurple);
        // Rectangle showing camera view
        // This loops through all the contours and draw points on the canvas

        for (MatOfPoint point : contourCopyPurple) {
            Point[] contourArray = point.toArray();
            // This Extracts the contour points and iterates through them.
            for (Point p : contourArray) {
                canvas.drawPoint((float) (p.x * scaleBmpPxToCanvasPx), (float) (p.y * scaleBmpPxToCanvasPx), contourpaintPurple);
            }
        }

        List<MatOfPoint> contourCopyGreen = new ArrayList<>(contoursGreen);
        // Rectangle showing camera view
        // This loops through all the contours and draw points on the canvas

        for (MatOfPoint point : contourCopyGreen) {
            Point[] contourArray = point.toArray();
            // This Extracts the contour points and iterates through them.
            for (Point p : contourArray) {
                canvas.drawPoint((float) (p.x * scaleBmpPxToCanvasPx), (float) (p.y * scaleBmpPxToCanvasPx), contourpaintGreen);
            }
        }

        if(targetDetected) {
            if (largestGreenRect != null) {
                 canvas.drawRect(makeGraphicsRect(largestGreenRect, scaleBmpPxToCanvasPx), ballPaintGreen);
            }
            if (largestPurpleRect != null) {
                canvas.drawRect(makeGraphicsRect(largestPurpleRect, scaleBmpPxToCanvasPx), ballPaintPurple);
            }
        }

        canvas.drawLine(IMG_WIDTH/2, 0, IMG_WIDTH/2, IMG_HEIGHT, linePaint);
        canvas.drawLine(0, IMG_HEIGHT/2, IMG_WIDTH, IMG_HEIGHT/2, linePaint);

    }
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // This code runs when you set up the camera processor
        // I mostly just leave this bit alone

        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        // More dashboard setup
    }
    public double[] getCenterpixel(boolean trueForBGRinsteadOfRGB) {
        centerPixelColorBGR = maskPurple.get(IMG_WIDTH / 2, IMG_HEIGHT / 2);
        centerPixelColorRGB = maskGreen.get(IMG_WIDTH / 2, IMG_HEIGHT / 2);
        if(trueForBGRinsteadOfRGB){
            return centerPixelColorBGR;
        }else{
            return centerPixelColorRGB;
        }
    }
    public Rect getObjectsDetected(ArrayList<MatOfPoint> contours){
        if (!contours.isEmpty()) {
            MatOfPoint largestContour = findLargestContour(contours);
          //  MatOfPoint2f largestContour2f = new MatOfPoint2f();
            if (largestContour != null) {

                List<Rect> rects = new ArrayList<>();
                for (int i = 0; i < contours.size(); i++) {
                    Rect rect = boundingRect(contours.get(i));
                    rects.add(rect);
                }
                if (!rects.isEmpty()) {
                    this.largestRect = VisionUtils.sortRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rects).get(0);
                    targetDetected = true;
                } else {
                    largestRect = null;
                    targetDetected = false;
                }}
                // Find the minimum enclosing circle for the largest contour
               /* largestContour.convertTo(largestContour2f, CvType.CV_32FC2);
                List<Circle> circles = new ArrayList<>();
                for (int i = 0; i < contours.size(); i++) {
                    Point center = new Point();
                    float[] radius = new float[(int) (Math.sqrt(( Imgproc.contourArea(contours.get(i)))/Math.PI))];
                    circleX =
                    circleY = center.y;
                    circleRadius = radius[0];
                    Imgproc.minEnclosingCircle(new MatOfPoint2f(largestContour2f), center, radius);
                    circles.add(circle);
                }*/
                /*Point center = new Point();
                float[] radius = new float[1];
                circleX = center.x;
                circleY = center.y;
                circleRadius = radius[0];
                Imgproc.minEnclosingCircle(new MatOfPoint2f(largestContour2f), center, radius);
                */

                // Update the public variables with the circle's properties

                /*if(largestCircle!=null){
                targetDetected = true;}
                // Draw the bounding circle on the original frame
                //  Imgproc.circle(input, center, (int) radius[0], new Scalar(255, 0, 0), 2);
                // Imgproc.circle(input, center, 5, new Scalar(0, 255, 0), -1); // Draw a dot at the center
            }} else {
                targetDetected = false;
    }*/

                drawContours(output, contours, -1, lightBlue);
                // Draws contours around shapes
                BaseOpMode.addData("target  detected", targetDetected);
                return largestRect;
            }else{
            return null;
    }}
    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        return largestContour;
    }

    public static double getError(boolean trueIfGreen){
        if(targetDetected){
            double centerBlob = 0;
            if(trueIfGreen&&largestGreenRect!=null){
                centerBlob= largestGreenRect.x + (largestGreenRect.width/2);
            } else if (largestPurpleRect!=null){
                centerBlob = largestPurpleRect.x+largestPurpleRect.width/2;};
                //error 157
            return (IMG_WIDTH / 2)  - centerBlob;
        }else{
            return 0;
        }
    }

    public static double getWidth(boolean trueIfGreen){
        if(targetDetected){

            if(trueIfGreen&&largestGreenRect!=null) {
                return largestGreenRect.width;

            }else if(largestPurpleRect!=null) {
                return largestPurpleRect.width;
            }else return 0;
        }else{
            return 0;
        }
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

}
