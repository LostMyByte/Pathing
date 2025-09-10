package org.firstinspires.ftc.teamcode.Motion.Controllers;

import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;
import com.google.gson.stream.JsonReader;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectOutputStream;

public class MPCPath {

    public static class MPCParams {
        public double QX = 100;
        public double QY = 100;
        public double QH = 100;
        public double QHV = 10;
        public double QVX = 10;
        public double QVY = 10;

        public double QFX = 20;
        public double QFY = 20;
        public double QFH = 20;
        public double QFVX = 10;
        public double QFVY= 10;
        public double QFHV = 10;

        public double R = 50;

        public double lr = 1.01;
        public double lambdaMax = 10000;
    }

    Vector start;
    double horizonTime;
    double threshold;
    double resolution;


    MPCParams params;

    ReferenceSignal referenceSignal;
    Signal sensorSignal;

    ElapsedTime timer;

    MPC controller;

    public void setPath(ReferenceSignal referenceSignal) {
        this.referenceSignal = referenceSignal;
    }

    public void setParams(MPCParams params) {
        this.params = params;
    }

    public void continueFrom(MPCPath previous) {
        this.start = previous.referenceSignal.target();
    }

    public void setTarget(double x, double y, double h, double vx, double vy, double vh) {
        this.referenceSignal = new ConstantSignal(new Vector(new double[] {x, y, h, vx, vy, vh}));
    }

    public void setStart(double x, double y, double h, double vx, double vy, double vh) {
        this.start = new Vector(new double[] {x, y, h, vx, vy, vh});
    }

    public void setMoveTime(double time) {
        this.horizonTime = time;
    }

    public void setAccuracy(double accuracy) {
        this.threshold = accuracy;
    }

    public void setResolution(double resolution) {
        this.resolution = resolution;
    }

    public void build() {
        Matrix Q = new GeneralMatrix(6, 6, new double[] {
                params.QX, 0, 0, 0, 0, 0,
                0, params.QY, 0, 0, 0, 0,
                0, 0, params.QH, 0, 0, 0,
                0, 0, 0, params.QVX, 0, 0,
                0, 0, 0, 0, params.QVY, 0,
                0, 0, 0, 0, 0, 0, params.QHV,
        });

        Matrix QF = new GeneralMatrix(6, 6, new double[] {
                params.QFX, 0, 0, 0, 0, 0,
                0, params.QFY, 0, 0, 0, 0,
                0, 0, params.QFH, 0, 0, 0,
                0, 0, 0, params.QFVX, 0, 0,
                0, 0, 0, 0, params.QFVY, 0,
                0, 0, 0, 0, 0, 0, params.QFHV,
        });

        Matrix R = Matrix.identityMatrix(3).multiplied(params.R);

        controller = new MPC(referenceSignal, sensorSignal, start, Q, R, QF, (int) (resolution*horizonTime), horizonTime, threshold, params.lr,  params.lambdaMax);
    }

    public void compile(int maxIter) {

        controller.iterate(3000, start);

    }

    public void start() {
        this.timer = new ElapsedTime();
    }

    public void save(String name) {

        JsonObject pathData = new JsonObject();

        JsonArray data = new JsonArray();

        for (int i = 0; i < controller.N; i++) {
            JsonObject step = new JsonObject();

            JsonArray xBytes = new JsonArray();
            JsonArray uBytes = new JsonArray();
            JsonArray kBytes = new JsonArray();
            JsonArray KBytes = new JsonArray();

            for (int control = 0; control < controller.numControls; control++) {
                uBytes.set(control, new JsonPrimitive(controller.currentControls[i].get(control)));
                kBytes.set(control, new JsonPrimitive(controller.k[i].get(control)));
            }

            for (int state = 0; state < controller.dimensions; state++) {
                xBytes.set(state, new JsonPrimitive(controller.currentTrajectory[i].get(state)));
                for (int control = 0; control < controller.numControls; control++) {
                    KBytes.set(control* controller.dimensions + state, new JsonPrimitive(controller.K[i].get(control, state)));
                }
            }

            step.add("x", xBytes);
            step.add("u", uBytes);
            step.add("k", kBytes);
            step.add("K", KBytes);

            data.set(i, step);
        }

        pathData.add("data", data);

        try {
            FileWriter writer = new FileWriter(name);
            writer.write(pathData.toString());
            writer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    public void load(String name) {

        Vector[] x = new Vector[controller.N];
        Vector[] u = new Vector[controller.N];
        Vector[] k = new Vector[controller.N];
        Matrix[] K = new Matrix[controller.N];

        try {
            JsonObject pathData = new JsonParser().parse(new JsonReader(new FileReader(name))).getAsJsonObject();

            JsonArray data = pathData.getAsJsonArray("data");

            for (int i = 0; i < controller.N; i++) {
                JsonObject step = data.get(i).getAsJsonObject();

                JsonArray xBytes = step.getAsJsonArray("x");
                JsonArray uBytes = step.getAsJsonArray("u");
                JsonArray kBytes = step.getAsJsonArray("k");
                JsonArray KBytes = step.getAsJsonArray("K");

                double[] xdata = new double[controller.dimensions];
                double[] udata = new double[controller.numControls];
                double[] kdata = new double[controller.numControls];
                double[] Kdata = new double[controller.dimensions * controller.numControls];
                for (int control = 0; control < controller.numControls; control++) {
                    udata[control] = uBytes.get(control).getAsDouble();
                    kdata[control] = kBytes.get(control).getAsDouble();
                }

                for (int state = 0; state < controller.dimensions; state++) {
                    xdata[i] = xBytes.get(state).getAsDouble();
                    for (int control = 0; control < controller.numControls; control++) {
                        Kdata[control*controller.dimensions + state] = KBytes.get(control*controller.dimensions + state).getAsDouble();
                    }
                }
            }

            controller.loadFromArray(x, u, k, K);

        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }

    public Vector getCorrection() {

        if (timer == null) start();
        double time = timer.time();

        int numDim = this.sensorSignal.getLength();
        Vector sensorData = Vector.length(numDim * 2);

        for (int i = 0; i < numDim; i++) {
            sensorData.put(i, sensorSignal.getIntegralVector().get(i));
            sensorData.put(i+numDim, sensorSignal.getDataVector().get(i));
        }
        Vector target = controller.getInterpolatedX(time);
        BaseOpMode.addData("TX", target.get(0));
        BaseOpMode.addData("TY", target.get(1));
        BaseOpMode.addData("TH", target.get(2));
        BaseOpMode.addData("TVX", target.get(3));
        BaseOpMode.addData("TVY", target.get(4));
        BaseOpMode.addData("TVH", target.get(5));

        sensorData.subtract(target);
        Vector correction = controller.getInterpolatedU(time);
        return correction.added(controller.getInterpolatedK(time).multiplied(sensorData));
    }


}
