/*
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.SensorNoise;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

package org.firstinspires.ftc.teamcode.teamcode.Motion.Localization;

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

public class Location extends SensorFusionEKF {

    OdoPodData odoPods;

    public Location(double startx, double starty, double starth) {
        super(new Vector(startx, starty, starth, 0, 0), new Signal[]{
                new OdoPodData(startx, starty, starth),
        }, new SensorModel[]{
                new OdoPodsSensorModel()
        }, new TankDrive());

        odoPods = (OdoPodData) this.sources[0];

        this.covarience = new GeneralMatrix(5, 5, new double[] {
                SensorNoise.model_pos, 0, 0, 0, 0,
                0, SensorNoise.model_pos, 0, 0, 0,
                0, 0, SensorNoise.model_h, 0, 0,
                0, 0, 0, SensorNoise.model_v, 0,
                0, 0, 0, 0, SensorNoise.model_vh,
        });
    }

    public double getPosX() {
        return state.get(0);
    }

    public double getPosY() {
        return state.get(1);
    }

    public double getPosH() {
        return state.get(2);
    }

    */
/**
     * Get the odometry angle wrapped within [0, 2pi]
     * @return
     *//*

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
            return state.get(3);
        }

        public double getVelH() {
            return state.get(4);
        }

        public Vector getTranslationalVelocity() {
            return new Vector(-state.get(3) * Math.sin(state.get(2)), state.get(3) * Math.cos(state.get(2)));
        }

        public Vector getPositionForTankDrive() {
            return state;
        }

        public Vector getPosition() {
            return getDataVector();
        }

        public void setPosition(double x, double y, double h) {
            this.state.put(0,x);
            this.state.put(1,y);
            this.state.put(2,h);
            odoPods.setPosition(x, y, h);
            this.P = new GeneralMatrix(5, 5);

        }

        public void updateOdoOffsets() {
            odoPods.updateOffsets();
        }

        @Override
        public void telemetry() {
            BaseOpMode.addData("Filtered X", state.get(0));
            BaseOpMode.addData("Filtered Y", state.get(1));
            BaseOpMode.addData("Filtered H", state.get(2));
            BaseOpMode.addData("Filtered V", state.get(3));
            BaseOpMode.addData("Filtered HV", state.get(4));

            for (Signal s: sources) {
                s.telemetry();
            }

            this.covarience = new GeneralMatrix(5, 5, new double[] {
                    SensorNoise.model_pos, 0, 0, 0, 0,
                    0, SensorNoise.model_pos, 0, 0, 0,
                    0, 0, SensorNoise.model_h, 0, 0,
                    0, 0, 0, SensorNoise.model_v, 0,
                    0, 0, 0, 0, SensorNoise.model_vh,
            });

        }

}
*/
