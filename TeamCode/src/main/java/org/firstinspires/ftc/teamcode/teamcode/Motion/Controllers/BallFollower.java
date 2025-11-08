package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.focalLengthMM;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.fx;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ConstantSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.PartialSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.VisionPID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallChaser;

@Config
public class BallFollower extends Controller {
    /**
     * A class to represent the error of a ball's distance while tracking.
     * Error Vector is [heading, distance]
     */
    private class ArtifactPositionSignal extends Signal {

        public ArtifactPositionSignal() {
            super(2);
        }

        @Override
        protected void update() {
            this.data = new Vector(BallChaser.getWidth(color == BallColor.GREEN), BallChaser.getError(color == BallColor.GREEN));
        }

        @Override
        public void telemetry() {
            BaseOpMode.addData("Ball angle", this.data.get(1));
            BaseOpMode.addData("Ball distance", this.data.get(0));
        }
    }

    public enum BallColor {
        GREEN,
        PURPLE
    }

    public static double halpha = 1;
    public static double dalpha = 1;
    BallColor color = BallColor.PURPLE;

    PID headingPID;
    PID distancePID;

    public BallFollower(BallColor color, double targetDistance) {
        super(new ConstantSignal(new Vector(0, targetDistance)), new ConstantSignal(new Vector(0, 0)));
        this.color = color;
        super.sensorSignal = new ArtifactPositionSignal(); // Have to do it like this because Java is weird

        headingPID = new PID(new ConstantSignal(new Vector(160)), new LowPassFilter(new PartialSignal(1, sensorSignal), halpha), VisionPID.headingPID);
        distancePID = new PID(new ConstantSignal(new Vector(targetDistance)), new LowPassFilter(new PartialSignal(0, sensorSignal), dalpha), VisionPID.drivePID);
    }

    @Override
    public Vector getCorrection() {
        double hcorrection = headingPID.getCorrection().get(0);
        BaseOpMode.addData("hcorrection", hcorrection);
        double dcorrection = distancePID.getCorrection().get(0);
        BaseOpMode.addData("dcorrection", dcorrection);
        return new Vector(dcorrection + hcorrection, dcorrection - hcorrection);
    }

    public double distance(double widthPixels){
        //double angleDeg = ((120*widthPixels)/320) /2;
        // angleRad = angleDeg * (PI/180);
        double diameterOfObject = 12.7/100; //in meters
        double distance;
        distance = diameterOfObject*fx/widthPixels-focalLengthMM;
        //distance = pixelsToMeters*widthPixels; //this will never work, but it's a neat idea
        //not real yet, ran out of time
        //real diameter times focal length in px over pixel diameter minus focal length MM
        //distance = Math.sqrt(Math.pow(30/Math.tan(angleRad),2)-(Math.pow(height,2)));
        return distance;
    }
}
