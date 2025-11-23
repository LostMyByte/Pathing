// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;
import com.google.gson.stream.JsonReader;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing.MPCTest;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Paths.Obstacle;
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

public class MPCPath extends Controller{

    @Config
    public static class MPCSettings {
        public static boolean fullEndCorrection = true;
        public static boolean invertHScale = true;
        public static boolean invertdSdU = true;
        public static boolean voltageCorrection = false;
    }
    VoltageSensor voltage;

    /**
     * A set of parameters to use for Model Predictive Control
     */
    public static class MPCParams {
        public double QX = 100; // Ongoing position cost in X
        public double QY = 100; // Ongoing position cost in Y
        public double QH = 100; // Ongoing position cost in Heading
        public double QV = 10;  // Ongoing Velocity cost
        public double QHV = 10; // Ongoing Heading Velocity cost

        public double QFX = 20; // Final Position cost in X
        public double QFY = 20; // Final Position cost in Y
        public double QFH = 20; // Final Position cost in Heading
        public double QFV = 10; // Final Velocity cost
        public double QFHV = 10; // Final Heading Velocity cost 

        public double R = 50; // Control cost

        public double lr = 2; // Learning rate
        public double lambdaMax = 10000000; // Max lambda (for descent)

        public double voltage = 13;
    }

    /**
     * States the controller can be in
     */
    public enum ControllerStates {
        Building, // Still need to call .build()
        Ready,    // Can start using
        Active,   // Actively moving along the path
        Finished  // Finished path and now using best-guess to preserve position
    }

    private ControllerStates state = ControllerStates.Building;

    
    Vector start; // Initial position 
    MPCPath continuationOf; // Path that comes before (optional). Will use that path's landing position as start to prevent cummulative errors
    double horizonTime; // How long to simulate path for
    double threshold;   // Quality of path; when to assume path has converged
    double resolution;  // How many timesteps per second to simulate.


    MPCParams params; // Parameters to use for path generation

    SystemModel model; // Drive train model

    String name; // Name of path for telemetry + file name

    //ReferenceSignal referenceSignal; // Optional; what the target trajectory should look like, roughly.
    //Signal sensorSignal; // Incoming data; optional, can also manually pass in state

    ElapsedTime timer;  // How long the path has been running for

    public MPC controller; // The actual path/controller object to use

    // These two are NOT related
    double startTime = 0; // When updating the path midway (MPC), when was the update done? Used for timers.
    double stopTime = 0;  // When to stop correcting for error in the strafe direction and only worry about heading + drive. Helps prevent late jitter.

    /**
     * Sets the target path to follow
     * @param referenceSignal
     */
    public void setPath(ReferenceSignal referenceSignal) {
        this.referenceSignal = referenceSignal;
    }

    public void setSensor(Signal sensorSignal) {
        this.sensorSignal = sensorSignal;
    }

    /**
     * Sets the Parameters to use in path generation
     * @param params
     */
    public void setParams(MPCParams params) {
        this.params = params;
    }

    /**
     * Continue from another path for improved accuracy.
     * @param previous  Path to continue from
     */
    public void continueFrom(MPCPath previous) {
        this.continuationOf = previous;
    }

    /**
     * Set the target state
     * @param x x position
     * @param y y position
     * @param h heading
     * @param v velocity
     * @param vh angular velocity
     */
    public void setTarget(double x, double y, double h, double v, double vh) {
        this.referenceSignal = new ConstantSignal(new Vector(new double[] {x, y, h, v, vh}));
    }

    /**
     * Set the position the path starts from
     * @param x x position
     * @param y y position
     * @param h heading
     * @param v velocity
     * @param vh angular velocity
     */
    public void setStart(double x, double y, double h, double v, double vh) {
        this.start = new Vector(new double[] {x, y, h, v, vh});
    }

    public void addObstacle(Obstacle obstacle) {
        controller.addObstacle(obstacle);
    }
    /**
     * How much time to simulate a control for. Should be about how long it takes to get to the target state.
     * @param time
     */
    public void setMoveTime(double time) {
        this.horizonTime = time;
    }

    /**
     * Sets when to stop making minor adjustments to the path
     * @param accuracy
     */
    public void setAccuracy(double accuracy) {
        this.threshold = accuracy;
    }

    /**
     * Sets how long to not correct for strafe errors. Prevents late-path jitters.
     * @param time
     */
    public void setStopTime(double time) {
        this.stopTime = time;
    }

    /**
     * How many iterations per timestep to simulate
     * @param resolution
     */
    public void setResolution(double resolution) {
        this.resolution = resolution;
    }

    /**
     * Get the current state of the controller
     * @return
     */
    public ControllerStates getState() {
        return state;
    }

    /**
     * Build the controller. Should always be called before use, even if loading a path.
     */
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

        controller = new MPC(referenceSignal, start, Q, R, QF, (int) (resolution*horizonTime), horizonTime, threshold, params.lr,  params.lambdaMax, model);

        this.voltage = BaseOpMode.hardware.voltageSensor.iterator().next();


    }

    /**
     * Compile the path relative to the current trajectory
     * @param maxIter   Maximum number of iterations. Prevents getting stuck in a loop.
     */
    public void compile(int maxIter) {

        BaseOpMode.addData("Compiling Path", name);
        controller.iterate(maxIter, start);

    }

    /**
     * Sets the system model
     * @param model
     */
    public void setModel(SystemModel model) {
        this.model = model;
    }

    /**
     * Sets the name of the path to use for saving to a file and telemetry
     * @param name
     */
    public void setName(String name) {
        this.name = name + ".json";
    }

    /**
     * (re)Start the path timer. Optional; will automatically be called by getCorrection() when controller is inactive.
     */
    public void start() {
        this.timer = new ElapsedTime();
        this.state = ControllerStates.Active;
    }


    /**
     * Updates the path based off a current position, MPC-style. Results in performance issues and (currently) a little buggy.
     * @param currentPosition   The current position of the robot
     * @param amount            How many iterations of updates to do.
     */
    public void update(Vector currentPosition, int amount) {
        double time = timer.time() - startTime;
        controller.stepForwardHorizon(currentPosition, time, amount);
    }


    /**
     * Get the target correction based off the sensor signal.
     * @return
     */
    public Vector getCorrection() {


        Vector sensorData = model.toStateSpace(sensorSignal);

        return getCorrection(sensorData);
    }

    public Vector getStateError(Vector position, double time) {
        Vector target = controller.getInterpolatedX(time);

        if (state == ControllerStates.Finished) {
            target = this.referenceSignal.target();
        }

        BaseOpMode.addData("Position Cost", controller.costFunction(target, referenceSignal.target(), new Vector(0,0), 1/resolution));
        BaseOpMode.addData("TX", target.get(0));
        BaseOpMode.addData("TY", target.get(1));
        BaseOpMode.addData("TH", target.get(2));
        BaseOpMode.addData("TV", target.get(3));
        BaseOpMode.addData("TVH", target.get(4));

        return position.subtracted(target);
    }

    public Vector getStateError(Vector position) {
        return getStateError(position, timer.time());
    }

    /**
     * Gets the correction given a state and a time along the path.
     * @param sensorData    Current state
     * @param time          Path time
     * @return
     */
    public Vector getCorrection(Vector sensorData, double time) {

        if (time > horizonTime) this.state = ControllerStates.Finished;

        // TODO: MAke work for not tank drive
        Vector loopback = TankDrive.getLoopback(sensorData);

        sensorData = getStateError(sensorData, time);

        Vector correction = controller.getInterpolatedU(time);

        if (time > horizonTime - startTime) {
            Vector posError = new Vector(sensorData.get(0), sensorData.get(1));
            Vector heading = new Vector(-Math.sin(sensorData.get(2)), Math.cos(sensorData.get(2)));
            posError = heading.multiplied(heading.dotProduct(posError));

            if (!MPCSettings.fullEndCorrection) {
                sensorData.put(0, posError.get(0));
                sensorData.put(1, posError.get(1));
            }
            correction.add(new Vector(heading.dotProduct(posError) * DriveWheels.Kp, posError.magnitude() * DriveWheels.Kp));
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

        Matrix K = model.dSdU(correction);
        if (MPCSettings.invertdSdU) K = K.inverted();
        K = K.multiplied(controller.getInterpolatedK(time));

        sensorData = model.h(sensorData.multiplied(MPCSettings.invertHScale ? -1 : 1)).multiplied(sensorData);

        Vector modelResponse = K.multiplied(DriveWheels.strength).multiplied(sensorData);
        BaseOpMode.addData("MH", modelResponse.get(1)-modelResponse.get(0));
        BaseOpMode.addData("MV", modelResponse.get(1)+modelResponse.get(0));
        correction.add(modelResponse);

        correction.add(feedback.multiplied(sensorData).added(loopback));

        if (MPCSettings.voltageCorrection) correction.multiply(params.voltage/voltage.getVoltage());

        return correction;
    }

    /**
     * Gets the correction at the current path time with the given sensor data.
     * @param sensorData    Current measured state.
     * @return
     */
    public Vector getCorrection(Vector sensorData) {
        if (state == ControllerStates.Ready) start();
        double time, simTime;
        time = timer.time() - startTime;
        simTime = time;

        BaseOpMode.addData("Time", time);

        Vector data = sensorData;


        // For testing only
        // Simulate controller completing the path to get landing position
        // Slows down loop times
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

    @Override
    public Vector targetPositionError(){
        Vector target = referenceSignal.target();
        // TODO: Make better at not-drivetrains
        return new Vector(target.get(0), target.get(1), target.get(2)).subtracted(sensorSignal.getDataVector());
    }

    /**
     * Gets the feedforward (no feedback) correction at the current path time
     * @return
     */
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

    /**
     * Attempts to load the path based off the given name, to avoid computation costs.
     * Note: You still need to specify the other parameters, this just saves a costly call to compile().
     * @throws FileNotFoundException
     */
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

    /**
     * Saves the path to a file based on the name. Can be restored by load().
     */
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
