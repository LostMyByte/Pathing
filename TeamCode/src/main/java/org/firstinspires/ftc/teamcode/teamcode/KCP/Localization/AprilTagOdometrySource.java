package org.firstinspires.ftc.teamcode.teamcode.KCP.Localization;



import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.cx;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.cy;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.fx;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.fy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Vector;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;



public class AprilTagOdometrySource {

    // Camera Settings


    private Position cameraPosition = new Position(DistanceUnit.INCH,
            -4, -6, 5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, 0, 180, 0);

    private boolean isValid; // Whether or not the current estimate is valid for use (i.e. not used yet).


    // Alpha-Beta-Gamma Filter

    //Tunables
    @Config
    static class FilterParameters {
        public static double alpha = 1;
        public static double beta = 0.0;
        public static double gamma = 0.0;
    }

    private Matrix alpha = GeneralMatrix.identityMatrix(3);
    private Matrix beta = GeneralMatrix.identityMatrix(3);
    private Matrix gamma = GeneralMatrix.identityMatrix(3);


    // Initial Values
    public Vector xk; // Position
    public Vector vk = new Vector(0, 0, 0); // Velocity
    public Vector ak = new Vector(0, 0, 0); // Acceleration

    private double oldTime;
    private ElapsedTime timer;

    private AprilTagProcessor tags;


    private Constants.Team team;

    Matrix Transform;

    private static Vector BOARD_CENTER = new Vector(72, 72, 0);

    public AprilTagOdometrySource(double startX, double startY, double startH, String team) {

        this.xk = new Vector(startX, startY, startH);

        this.timer = new ElapsedTime();
        this.oldTime = timer.time();

        tags = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        if (team.toLowerCase() == "blue") {
            this.team = Constants.Team.BLUE;
            this.Transform = (new GeneralMatrix(3, 3, new double[]{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}));
        }
        else if (team.toLowerCase() == "red") {
            this.team = Constants.Team.RED;
            this.Transform = (new GeneralMatrix(3, 3, new double[]{0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0}));
        }
        else this.team = autoTeam(20);


    }


    public double[] getPosition() {
        return new double[]{xk.get(0), xk.get(1)};
    }

    public double getHeading() {
        return xk.get(2);
    }

    public Vector getValidPositionVector() {
        if (isValid) {
            isValid = false;
            return xk;
        }
        return null;
    }

    // Uses sparkFun positioning to make an estimate of the displacement
    // Then will use apriltags to improve upon that estimate
    public void update() { //SparkFunOTOS.Pose2D sparkFunPosition, SparkFunOTOS.Pose2D sparkFunVelocity, SparkFunOTOS.Pose2D SparkFunAcceleration) {

        ArrayList<AprilTagDetection> detections = tags.getDetections();

        if (detections == null || detections.isEmpty()) {
            return;
        }
        isValid = true;
        AprilTagDetection detection = detections.get(0);

        double time = timer.time();
        double deltaTime = time - oldTime;
        oldTime = time;

        Pose3D pose = detection.robotPose;

        double x = pose.getPosition().x;
        double y = pose.getPosition().y;
        double h = pose.getOrientation().getYaw();


        Matrix tempAlpha = alpha.multiplied(FilterParameters.alpha);
        Matrix tempBeta = alpha.multiplied(FilterParameters.beta);
        Matrix tempGamma = alpha.multiplied(FilterParameters.gamma);


        // TODO: Get heading from apriltags
        Vector zk = BOARD_CENTER.added(Transform.multiplied(new Vector(x, y,h)));

        BaseOpMode.addData("Tag Y", zk.get(1));
        // Can add data from sparkfun to use in estimation of xk
        // Can add data from accelerometer to help estimate ak
        xk = xk.added(vk.multiplied(deltaTime)).added(ak.multiplied(0.5 * deltaTime * deltaTime));
        vk = vk.added(ak.multiplied(deltaTime));


        Vector rk = zk.subtracted(xk);

        xk.add(tempAlpha.multiplied(rk));
        vk.add(tempBeta.multiplied(1.0 / deltaTime).multiplied(rk));
        ak.add(tempGamma.multiplied(1.0 / (2 * deltaTime * deltaTime)).multiplied(rk));

        BaseOpMode.addData("ay", ak.get(1));
        BaseOpMode.addData("vy", vk.get(1));
        BaseOpMode.addData("py", xk.get(1));
        // FOR TESTING: Just ignore filter
        //xk = zk;
    }


    public int getId(int index) {
        if (tags.getDetections().size() != 0) {
            return tags.getDetections().get(index).id;
        } else {
            return 0;
        }
    }


    public AprilTagDetection getClosestByID(int id) {
        List<AprilTagDetection> currentDetections = tags.getDetections();
        if (currentDetections == null || currentDetections.isEmpty()) {
            return null;
        }
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id) {
                return detection;
            }
//
        }
        return null;
    }


    public AprilTagProcessor getProcessor() {
        return tags;
    }

    public Constants.Team autoTeam(int numTries) {

        for (int i = 0; i<numTries; i++) {
            ArrayList<AprilTagDetection> tempList = tags.getDetections();
            if (!(tempList==null || tempList.isEmpty())) {
                Position position = tempList.get(0).robotPose.getPosition();

                // See if robot is on either team
                if (position.y > 0) return Constants.Team.BLUE;
                else return Constants.Team.RED;
            }
        }
        return null;
    }
}


