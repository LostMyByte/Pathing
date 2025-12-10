package org.firstinspires.ftc.teamcode.teamcode.Motion.Localization;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Filters.SensorFusionEKF;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.SensorModels.OdoPodsSensorModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.SensorModels.SensorModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.SensorNoise;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

@Config
public class Location extends Signal {

    OdoPodData odoPods;
    LimeLightData limeLight;

    Vector oldData;

    public static double llAlpha = 0.05;
    public Location(double startx, double starty, double starth) {
        super(3);
        odoPods = new OdoPodData(startx, starty, starth);
        limeLight = new LimeLightData();
        oldData = new Vector(startx,starty,starth);
        data = new Vector(startx,starty,starth);
    }

    public double getPosX() {
        return data.get(0);
    }

    public double getPosY() {
        return data.get(1);
    }

    public double getPosH() {
        return data.get(2);
    }

    /**
     * Get the odometry angle wrapped within [0, 2pi]
     * @return
     */
    public double getWrappedAngle() {
        double angle = getPosH();
        while (angle < 0) {
            angle += Math.PI * 2;
        }
        while (angle > 2* Math.PI) {
            angle -= Math.PI * 2;
        }
        return angle;
    }

    public double getVelDrive() {
        return odoPods.driveVelocity;
    }

    public double getVelH() {
        return odoPods.getGradient().get(2);
    }

    public Vector getTranslationalVelocity() {
        return new Vector(odoPods.driveVelocity * -Math.sin(getPosH()), odoPods.driveVelocity * Math.cos(getPosH()));
    }

    public Vector getPositionForTankDrive() {
        return new Vector(getPosX(), getPosY(), getPosH(), odoPods.driveVelocity, getVelH());
    }

    public Vector getPosition() {
        return getDataVector();
    }

    public void setPosition(double x, double y, double h) {
        data = new Vector(x, y, h);
        oldData = new Vector(x,y,h);
        odoPods.setPosition(x, y, h);

    }

    public void updateOdoOffsets() {
        odoPods.updateOffsets();
    }

    Matrix LLprojection = new GeneralMatrix(3, 3, new double[] {
            1, 0, 0,
            0, 1, 0,
            0, 0, 0,
    });
    @Override
    protected void update() {
        data.add(odoPods.getDataVector().subtracted(oldData));
        oldData = odoPods.getDataVector();
        LimeLightData.botHeading = data.get(2);
        if (limeLight.goodData) data.add(LLprojection.multiplied(limeLight.getDataVector().subtracted(data).multiplied(llAlpha)));
    }

    @Override
    public void telemetry() {
        BaseOpMode.addData("Filtered X", getPosX());
        BaseOpMode.addData("Filtered Y", getPosY());
        BaseOpMode.addData("Filtered H", getPosH());
        BaseOpMode.addData("Filtered V", getVelDrive());
        BaseOpMode.addData("Filtered HV", getVelH());

        odoPods.telemetry();
        limeLight.telemetry();
    }

    public void setPositionToLL() {
        odoPods.setPosition(limeLight.getDataVector());
    }
}
