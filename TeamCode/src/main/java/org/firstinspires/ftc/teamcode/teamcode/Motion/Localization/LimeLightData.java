package org.firstinspires.ftc.teamcode.teamcode.Motion.Localization;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.Team.BLUE;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.Team.RED;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.goalAngle;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.goalAprilTagHeight;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.limelightAngleOffset;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.limelightLensHeightFromGround;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.team;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.tyAlpha;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.tyFiltered;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.util.List;

public class LimeLightData extends Signal {


    @Config
    public static class LLDataFixParams{
        public static double piScale = -1;
        public static double yawScale = -1;
        public static double goalScale = -1;
        public static double turretScale = -1;
    }

    public boolean goodData;
    Limelight3A limelight;

    public static Vector rawData;


    public static double turretAngle = 0;
    public static double botHeading = 0;
    Matrix pitchCorrection = new GeneralMatrix(3, 3, new double[] {
        1, 0, 0,
        0, Math.cos(Math.toRadians(limelightAngleOffset)), -Math.sin(Math.toRadians(limelightAngleOffset)),
        0, Math.sin(Math.toRadians(limelightAngleOffset)), Math.cos(Math.toRadians(limelightAngleOffset)),
    });

    Matrix toFieldSpace;


    public LimeLightData() {
        super(3);
        switch (team) {
            case BLUE:
                toFieldSpace = new GeneralMatrix(2, 3, new double[] {
                        Math.cos(Math.toRadians(Constants.goalAngle)), -Math.sin(Math.toRadians(Constants.goalAngle)), 0,
                        -Math.sin(Math.toRadians(Constants.goalAngle)), -Math.cos(Math.toRadians(Constants.goalAngle)), 0
                });
                break;
            case RED:
                toFieldSpace = new GeneralMatrix(2, 3, new double[] {
                        Math.cos(Math.toRadians(-Constants.goalAngle)), -Math.sin(Math.toRadians(-Constants.goalAngle)), 0,
                        -Math.sin(Math.toRadians(-Constants.goalAngle)), -Math.cos(Math.toRadians(-Constants.goalAngle)), 0
                });
                break;
        }
        limelight = BaseOpMode.hardware.get(Limelight3A.class, "limelight");


        limelight.setPollRateHz(100); // make number higher to get more data
        limelight.pipelineSwitch(1);
        limelight.start();

        limelight.reloadPipeline();
    }

    public double getDistance() {
        return Math.sqrt(data.get(0)*data.get(0) + data.get(1)*data.get(1));
    }

    @Override
    protected void update() {

        double yaw = 0;
        goodData = false;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            BaseOpMode.addData("HybridDistance", "No valid tag");
            return;
        }

        List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
        if (fids == null || fids.isEmpty()) return;
        for (LLResultTypes.FiducialResult fid : fids)
            if ((fid.getFiducialId() == 20 && team == BLUE) || (fid.getFiducialId() == 24 && team == RED)) {
                goodData = true;

                //yaw = fid.getCameraPoseTargetSpace().getOrientation().getPitch(AngleUnit.RADIANS);
                //yaw -= Math.PI + Math.toRadians(goalAngle) - turretAngle;
                yaw = botHeading;
                double x = fid.getCameraPoseTargetSpace().getPosition().x;
                double y = fid.getCameraPoseTargetSpace().getPosition().y;
                double z = fid.getCameraPoseTargetSpace().getPosition().z;
                if (y < 0) {
                    y = -y;
                    x = -x;
                    yaw = data.get(2);
                    goodData = false;
                }

                Vector pos = pitchCorrection.multiplied(new Vector(-x, z, y)).multiplied(100);
                //BaseOpMode.addData("LL Height", pos.get(2));
                pos = toFieldSpace.multiplied(pos);
                rawData = new Vector(pos.get(0), pos.get(1));
                pos = handleOffsets(pos, yaw, turretAngle);

                /*BaseOpMode.addData("LL robot Pitch", fid.getCameraPoseTargetSpace().getOrientation().getPitch(AngleUnit.RADIANS));
                BaseOpMode.addData("LL robot Yaw", fid.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.DEGREES));
                BaseOpMode.addData("LL robot Roll", fid.getCameraPoseTargetSpace().getOrientation().getRoll(AngleUnit.DEGREES));
                */


                // Dynamic angle compensation beyond ~2.8 m
                // Start with baseline Limelight mount angle

                // Final smoothing filter for stability
                data = new Vector(pos.get(0), pos.get(1), yaw);



                ///Heading and telemetry
            }
    }

    public static Vector handleOffsets(Vector pos, double yaw, double turretAngle) {
        pos.add(new Vector((team ==BLUE ? 1 : -1) * Constants.goalAprilTagCornerDistanceX, Constants.goalAprilTagCornerDistanceY));
        //Turret radial offset
        pos.add(new Vector(-Math.sin(yaw + turretAngle), Math.cos(yaw + turretAngle)).multiplied(-Constants.LimeLightOffsetRadius));
        //Turret position offset
        pos.add(new Vector(Math.cos(yaw), Math.sin(yaw)).multiplied(Constants.TurretOffsetX));

        return pos;
    }
    @Override
    public void telemetry() {
        BaseOpMode.addData("Turret Angle", turretAngle);
        BaseOpMode.addData("LL X", data.get(0));
        BaseOpMode.addData("LL Y", data.get(1));
        BaseOpMode.addData("LL H", data.get(2));
    }
}
