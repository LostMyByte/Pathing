package org.firstinspires.ftc.teamcode.Motion.Controllers;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;
import com.google.gson.stream.JsonReader;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class MPCPath {

    public static class MPCParams {
        public double QX = 100;
        public double QY = 100;
        public double QH = 100;
        public double QHV = 10;
        public double QV = 10;

        public double QFX = 20;
        public double QFY = 20;
        public double QFH = 20;
        public double QFV = 10;
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

    SystemModel model;

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

    public void setTarget(double x, double y, double h, double v, double vh) {
        this.referenceSignal = new ConstantSignal(new Vector(new double[] {x, y, h, v, vh}));
    }

    public void setStart(double x, double y, double h, double v, double vh) {
        this.start = new Vector(new double[] {x, y, h, v, vh});
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
        Matrix Q = new GeneralMatrix(5, 5, new double[] {
                params.QX, 0, 0, 0, 0,
                0, params.QY, 0, 0, 0,
                0, 0, params.QH, 0, 0,
                0, 0, 0, params.QV, 0,
                0, 0, 0, 0, params.QHV,
        });

        Matrix QF = new GeneralMatrix(5, 5, new double[] {
                params.QFX, 0, 0, 0, 0,
                0, params.QFY, 0, 0, 0,
                0, 0, params.QFH, 0, 0,
                0, 0, 0, params.QFV, 0,
                0, 0, 0, 0, params.QFHV,
        });

        Matrix R = Matrix.identityMatrix(2).multiplied(params.R);

        controller = new MPC(referenceSignal, sensorSignal, start, Q, R, QF, (int) (resolution*horizonTime), horizonTime, threshold, params.lr,  params.lambdaMax, model);
    }

    public void compile(int maxIter) {

        controller.iterate(maxIter, start);

    }

    public void setModel(SystemModel model) {
        this.model = model;
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
                uBytes.add(new JsonPrimitive(controller.currentControls[i].get(control)));
                kBytes.add(new JsonPrimitive(controller.k[i].get(control)));
            }

            for (int state = 0; state < controller.dimensions; state++) {
                xBytes.add(new JsonPrimitive(controller.currentTrajectory[i].get(state)));
                for (int control = 0; control < controller.numControls; control++) {
                    KBytes.add(new JsonPrimitive(controller.K[i].get(control, state)));
                }
            }

            step.add("x", xBytes);
            step.add("u", uBytes);
            step.add("k", kBytes);
            step.add("K", KBytes);

            data.add(step);
        }

        pathData.add("data", data);

        name = "/storage/emulated/0/" + name;

        try {
            FileWriter writer = new FileWriter(name);
            writer.write(pathData.toString());
            writer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    public void load(String name) throws FileNotFoundException{

        name = "/storage/emulated/0/" + name;

        Vector[] x = new Vector[controller.N];
        Vector[] u = new Vector[controller.N];
        Vector[] k = new Vector[controller.N];
        Matrix[] K = new Matrix[controller.N];


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
                xdata[state] = xBytes.get(state).getAsDouble();
                for (int control = 0; control < controller.numControls; control++) {
                    Kdata[control*controller.dimensions + state] = KBytes.get(control*controller.dimensions + state).getAsDouble();
                }
            }

            x[i] = new Vector(xdata);
            u[i] = new Vector(udata);
            k[i] = new Vector(kdata);
            K[i] = new GeneralMatrix(controller.dimensions, controller.numControls, Kdata).transposed();

        }

            controller.loadFromArray(x, u, k, K);




    }


    public Vector getCorrection() {


        int numDim = this.sensorSignal.getLength();
        Vector sensorData = Vector.length(numDim * 2);

        for (int i = 0; i < numDim; i++) {
            sensorData.put(i, sensorSignal.getIntegralVector().get(i));
            sensorData.put(i+numDim, sensorSignal.getDataVector().get(i));
        }

        return getCorrection(sensorData);
    }
    public Vector getCorrection(Vector sensorData) {

        if (timer == null) start();
        double time = timer.time();

        Vector target = controller.getInterpolatedX(time);


        BaseOpMode.addData("TX", target.get(0));
        BaseOpMode.addData("TY", target.get(1));
        BaseOpMode.addData("TH", target.get(2));
        BaseOpMode.addData("TV", target.get(3));
        BaseOpMode.addData("TVH", target.get(4));


        sensorData.subtract(target);
        Vector correction = controller.getInterpolatedU(time);

        BaseOpMode.addData("FH", correction.get(1)-correction.get(0));
        BaseOpMode.addData("FV", correction.get(1)+correction.get(0));
        return model.controlLimit(correction.added(controller.getInterpolatedK(time).multiplied(sensorData)));
    }

    public Vector getFeedForward() {
        if (timer == null) start();
        double time = timer.time();

        Vector target = controller.getInterpolatedX(time);

        Vector correction = controller.getInterpolatedU(time);


        BaseOpMode.addData("TX", target.get(0));
        BaseOpMode.addData("TY", target.get(1));
        BaseOpMode.addData("TH", target.get(2));
        BaseOpMode.addData("TV", target.get(3));
        BaseOpMode.addData("TVH", target.get(4));

        BaseOpMode.addData("FH", correction.get(1)-correction.get(0));
        BaseOpMode.addData("FV", correction.get(1)+correction.get(0));
        return correction;
    }


}
