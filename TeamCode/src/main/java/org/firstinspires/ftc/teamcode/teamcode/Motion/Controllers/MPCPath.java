// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;
import com.google.gson.stream.JsonReader;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ConstantSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ReferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.io.File;
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

        public double lr = 2;
        public double lambdaMax = 10000000;
    }

    public enum ControllerStates {
        Building,
        Ready,
        Active,
        Finished
    }

    private ControllerStates state = ControllerStates.Building;;

    Vector start;
    MPCPath continuationOf;
    double horizonTime;
    double threshold;
    double resolution;


    MPCParams params;

    SystemModel model;

    String name;

    ReferenceSignal referenceSignal;
    Signal sensorSignal;

    ElapsedTime timer;

    MPC controller;

    // These two are NOT related
    double startTime = 0;
    double stopTime = 0;

    public void setPath(ReferenceSignal referenceSignal) {
        this.referenceSignal = referenceSignal;
    }

    public void setParams(MPCParams params) {
        this.params = params;
    }

    public void continueFrom(MPCPath previous) {
        this.continuationOf = previous;
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

    public void setStopTime(double time) {
        this.stopTime = time;
    }

    public void setResolution(double resolution) {
        this.resolution = resolution;
    }

    public ControllerStates getState() {
        return state;
    }

    public void build() {
        if (state != ControllerStates.Building) return;

        if (continuationOf != null) {
            this.start = continuationOf.controller.currentTrajectory[continuationOf.controller.currentTrajectory.length-1];
        }

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

        this.state = ControllerStates.Ready;

        controller = new MPC(referenceSignal, sensorSignal, start, Q, R, QF, (int) (resolution*horizonTime), horizonTime, threshold, params.lr,  params.lambdaMax, model);


    }

    public void compile(int maxIter) {

        BaseOpMode.addData("Compiling Path", name);
        controller.iterate(maxIter, start);

    }

    public void setModel(SystemModel model) {
        this.model = model;
    }

    public void setName(String name) {
        this.name = name + ".json";
    }

    public void start() {
        this.timer = new ElapsedTime();
        this.state = ControllerStates.Active;
    }



    public void update(Vector currentPosition, int amount) {
        double time = timer.time() - startTime;
        double currentNApprox = time * resolution;
        int currentNExact = ((int) (time * resolution));

        if (currentNExact >= 1) {
            Vector newstate = model.stateTransitionFunction(currentPosition, controller.getInterpolatedU(time), currentNExact - currentNApprox);

            controller.stepForwardHorizon(newstate, currentNExact);


            controller.iterate(amount, newstate);

            startTime = time;
        }
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
    public Vector getCorrection(Vector sensorData, double time) {

        if (time > horizonTime) this.state = ControllerStates.Finished;

        Vector target = controller.getInterpolatedX(time);

        if (state == ControllerStates.Finished) {
            target = this.referenceSignal.target();
        }

        BaseOpMode.addData("TX", target.get(0));
        BaseOpMode.addData("TY", target.get(1));
        BaseOpMode.addData("TH", target.get(2));
        BaseOpMode.addData("TV", target.get(3));
        BaseOpMode.addData("TVH", target.get(4));

        sensorData = sensorData.subtracted(target);
        Vector correction = controller.getInterpolatedU(time);

        if (time > horizonTime - startTime) {
            Vector posError = new Vector(sensorData.get(0), sensorData.get(1));
            Vector heading = new Vector(Math.cos(sensorData.get(2)), Math.sin(sensorData.get(2)));
            posError = heading.multiplied(heading.dotProduct(posError));

            sensorData.put(0, posError.get(0));
            sensorData.put(1, posError.get(1));
        }

        BaseOpMode.addData("FH", correction.get(1)-correction.get(0));
        BaseOpMode.addData("FV", correction.get(1)+correction.get(0));
        Matrix feedback = new GeneralMatrix(5, 2, new double[] {
                0, 0,
                0, 0,
                DriveWheels.Kih, -DriveWheels.Kih,
                DriveWheels.Kpv, DriveWheels.Kpv,
                DriveWheels.Kvh, -DriveWheels.Kvh,
        }).transposed();

        Matrix K = model.dSdU(correction).inverted().multiplied(controller.getInterpolatedK(time));

        sensorData = model.h(sensorData.multiplied(-1)).multiplied(sensorData);

        correction.add(K.multiplied(DriveWheels.strength).multiplied(sensorData));

        return correction.added(feedback.multiplied(sensorData)).added(TankDrive.getLoopback(target));
    }

    public Vector getCorrection(Vector sensorData) {
        if (state == ControllerStates.Ready) start();
        double time, simTime;
        time = timer.time() - startTime;
        simTime = time;

        BaseOpMode.addData("Time", time);

        Vector data = sensorData;


        /*while (simTime < horizonTime) {
            sensorData = model.stateTransitionFunction(sensorData, getCorrection(sensorData, simTime),Signal.deltaTime);
            simTime += Math.max(Signal.deltaTime, 0.01);
        }*/

        Vector correction = getCorrection(data, time);

        BaseOpMode.addData("LX", sensorData.get(0));
        BaseOpMode.addData("LY", sensorData.get(1));
        BaseOpMode.addData("LH", sensorData.get(2));
        BaseOpMode.addData("LV", sensorData.get(3));
        BaseOpMode.addData("LVH", sensorData.get(4));

        return correction;
    }

    public Vector getFeedForward() {
        if (state == ControllerStates.Ready) start();
        double time = timer.time() - startTime;

        if (time > horizonTime) this.state = ControllerStates.Finished;

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

    public void load() throws FileNotFoundException{

        String path = "/storage/emulated/0/" + name;

        Vector[] x = new Vector[controller.N];
        Vector[] u = new Vector[controller.N];
        Vector[] k = new Vector[controller.N];
        Matrix[] K = new Matrix[controller.N];


        JsonObject pathData = new JsonParser().parse(new JsonReader(new FileReader(path))).getAsJsonObject();

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

    public void save() {

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

        String path = "/storage/emulated/0/" + name;

        try {
            File file = new File(path);
            if (!file.exists()) file.createNewFile();
            FileWriter writer = new FileWriter(file);
            writer.write(pathData.toString());
            writer.close();

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

}
