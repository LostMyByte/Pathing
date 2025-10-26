// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.EigenOps_DDRM;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ReferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

/**
 * A general-purpose Differential Dynamic Programming-based Model Predictive Control similar to an iLQR scheme.
 * Generates a time-varying feedback controller, as well as a predicted state and control trajectory.
 **/
public class MPC {

    public Matrix Q; // Distance cost
    public Matrix R; // Control cost
    public Matrix QF; // Terminal Distance Cost

    private double lr; // Learning Rate
    private double lambda = 1; // Current step size
    private double lambda_max = 1000;
    private double threshold; // Threshold for path being considered good


    public Vector[] currentControls; // Current Control Trajectory
    public Vector[] k; // Current feedforward adjustment from the current trajectory. Should be near-zero when path is optimal.
    public Matrix[] K; // Current time-varying feedback controller.
    public Vector[] currentTrajectory; // Current state trajectory.


    public ReferenceSignal referenceSignal; // Target trajectory

    public int dimensions; // Dimensions of state space
    public int numControls; // Dimensions of control space

    public int N; // Number of timesteps to simulate
    private double horizon; // Horizion time (time ahead to simulate) in seconds
    private double dt; // Timestep
    
    private SystemModel model; // Model of system dynamics


    /**
     * Create class without compiling path.
     * @param referenceSignal   Target trajectory to follow
     * @param start             Start state
     * @param Q                 Ongoing error cost
     * @param R                 Control Cost
     * @param QF                Final Error Cost
     * @param N                 Timesteps to simulate
     * @param time              Estimated path duration (Horizon time)
     * @param threshold         Relative improvement threshold before path is considered finished.
     * @param lr                Learning rate for descent
     * @param lambda_max        Max learning rate
     * @param model             Model of system Dynamics
     */
    public MPC(ReferenceSignal referenceSignal, Vector start, Matrix Q, Matrix R, Matrix QF, int N, double time, double threshold, double lr, double lambda_max, SystemModel model) {
        this.referenceSignal = referenceSignal;

        this.lambda_max = lambda_max;
        this.dimensions = referenceSignal.getLength();
        this.numControls = 2;
        this.horizon = time;

        this.lr = lr;

        this.Q  = Q;
        this.R  = R;
        this.QF = QF;

        this.N = N;

        this.model = model;

        this.dt = horizon/N;
        initializeControls(N, start);

        this.threshold = threshold;

        this.lambda = 1;
    }


    /**
     * Generates currentTrajectory based off currentControls and start state
     * @param startState    Position in state space to start from
     */
    public void generateTrajectory(Vector startState) {

        this.currentTrajectory = new Vector[N];
        Vector currentState = new Vector(startState.getData());

        for (int i = 0; i < N; i++) {
            currentTrajectory[i] = currentState;

            // Simulate Linearized dynamics
            currentState = model.stateTransitionFunction(currentState, currentControls[i], dt);
        }
    }

    /**
     * Cummulative cost function for current path
     * @return  total cost
     */
    public double getTotalCost() {
        double currentCost = 0;

        for (int i =0; i < N; i++) {
            // Add to cost function
            // TODO: Handle non-constant reference signals
            currentCost += costFunction(currentTrajectory[i], referenceSignal.target(), currentControls[i], dt);
        }

        // Add terminal cost
        Vector finalState = currentTrajectory[currentTrajectory.length-1];
        currentCost += finalState.dotProduct(QF.multiplied(finalState));

        return currentCost;
    }

    /**
     * Ongoing Cost Function for a given timestep
     * @param state     Current state
     * @param target    Target State
     * @param control   Active Control
     * @param time      How long before a new state
     * @return Momentary cost
     */
    private double costFunction(Vector state, Vector target, Vector control, double time) {
        Vector error = target.subtracted(state);
        return (error.dotProduct(Q.multiplied(error)) + control.dotProduct(R.multiplied(control)))* time;
    }

    /**
     * Derivative of Ongoing Cost Function with respect to state
     * @param x         Current state
     * @param target    Target State
     * @param time      How long before a new state
     * @return Momentary cost gradient
     */
    private Vector dCdX(Vector x, Vector target, double time) {
        return Q.multiplied(target.subtracted(x)).multiplied(-2 * time);
    }

    /**
     * Derivative of Final Cost Function with respect to state
     * @param x         Final state
     * @param target    Target State
     * @return Momentary cost gradient
     */
    private Vector dCFdX(Vector x, Vector target) {
        return QF.multiplied(target.subtracted(x)).multiplied(-2);
    }

    /**
     * Derivative of Ongoing Cost Function with respect to control
     * @param u         Current control
     * @param time      How long before a new state
     * @return Momentary cost gradient
     */
    private Vector dCdU(Vector u, double time) {
        return R.multiplied(u).multiplied(2*time);
    }

    /**
     * Second Derivative of Ongoing Cost Function with respect to state
     * @param time      How long before a new state
     * @return Momentary cost second derivative as a Jacobian
     */
    private Matrix dCdX2(double time) {
        return Q.multiplied(2 * time);
    }

    /**
     * Second Derivative of Final Cost Function with respect to state
     * @return Final cost second derivative as a Jacobian
     */
    private Matrix dCFdX2() {
        return QF.multiplied(2);
    }

    /**
     * Second Derivative of Ongoing Cost Function with respect to control
     * @param time      How long before a new state
     * @return Momentary cost second derivative as a Jacobian
     */
    private Matrix dCdU2(double time) {
        return R.multiplied(2 * time);
    }

    /**
     * Initialize current trajectory with static controls
     * @param N         How many timesteps to generate
     * @param start     Start state
     */
    private void initializeControls(int N, Vector start) {
        this.currentControls = new Vector[N];
        this.currentTrajectory = new Vector[N];
        this.currentTrajectory[0] = start;
        this.k = new Vector[N];
        this.K = new Matrix[N];
        for (int i = 0; i < N; i++) {
            currentControls[i] = Vector.withValue(0, numControls);

            this.k[i] = Vector.length(2);
            this.K[i] = new GeneralMatrix(2, 5);
            if (i != N-1) {
                currentTrajectory[i+1] = model.stateTransitionFunction(currentTrajectory[i], currentControls[i], dt);
            }
        }

    }

    /**
     * Get a linearly interpolated feedforward control change Vector at a given timestep.
     * This is not the feed forward term to use when executing a path.
     * @param time  What time to interpolate for
     * @return      Interpolated Vector
     */
    public Vector getInterpolatedk(double time) {
        return getInterpolatedk(time, horizon);
    }

    /**
     * Get a linearly interpolated feed-forward Vector at a given timestep.
     * Use this to get the feed-forward control for a path.
     * @param time  What time to interpolate for
     * @return      Interpolated Vector
     */
    public Vector getInterpolatedU(double time) {
        return getInterpolatedU(time, horizon);
    }

    /**
     * Get a linearly interpolated target state Vector at a given timestep.
     * @param time  What time to interpolate for
     * @return      Interpolated Vector
     */
    public Vector getInterpolatedX(double time) {
        return getInterpolatedX(time, horizon);
    }

    /**
     * Get a linearly interpolated feedback matrix at a given timestep.
     * @param time  What time to interpolate for
     * @return      Interpolated Matrix
     */
    public Matrix getInterpolatedK(double time) {
        return getInterpolatedK(time, horizon);
    }

    /**
     * Get a linearly interpolated feedforward control change Vector at a given timestep.
     * Treats controls as being spread out over a custom horizon time. Only used internally.
     * This is not the feed forward term to use when executing a path.
     * @param time  What time to interpolate for
     * @return      Interpolated Vector
     */
    private Vector getInterpolatedk(double time, double horizon) {
        if (time >= (horizon-2 * dt)) return k[N-2];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (k[index].multiplied(1 - alpha).added(k[index + 1].multiplied(alpha)));
    }

    /**
     * Get a linearly interpolated feedback matrix at a given timestep.
     * Treats controls as being spread out over a custom horizon time. Only used internally.
     * @param time  What time to interpolate for
     * @return      Interpolated Matrix
     */
    private Matrix getInterpolatedK(double time, double horizon) {
        if (time >= (horizon-2* dt)) return K[N-2];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (K[index].multiplied(1 - alpha).added(K[index + 1].multiplied(alpha)));
    }
    /**
     * Get a linearly interpolated target state Vector at a given timestep.
     * Treats controls as being spread out over a custom horizon time. Only used internally.
     * @param time  What time to interpolate for
     * @return      Interpolated Vector
     */
    private Vector getInterpolatedX(double time, double horizon) {
        if (time >= (horizon-dt)) return currentTrajectory[N-1];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (currentTrajectory[index].multiplied(1 - alpha).added(currentTrajectory[index + 1].multiplied(alpha)));
    }

    /**
     * Get a linearly interpolated feed-forward Vector at a given timestep.
     * Treats controls as being spread out over a custom horizon time. Only used internally.
     * Use this to get the feed-forward control for a path.
     * @param time  What time to interpolate for
     * @return      Interpolated Vector
     */
    private Vector getInterpolatedU(double time, double horizon) {
        if (time >= (horizon-2*dt)) return Vector.length(numControls);
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (currentControls[index].multiplied(1 - alpha).added(currentControls[index + 1].multiplied(alpha)));
    }

    /**
     * Updates the current control, state, and feedback trajectories based off a predicted improvement.
     * @param currentState  What the current starting state is. Often currentTrajectory[0].
     */
    private void updateControls(Vector currentState) {

        // First Derivative of "Value" Function: How much a change in state will change the future cost.
        Vector vx = dCFdX(currentTrajectory[N-1], referenceSignal.target());
        Matrix vxx = dCFdX2(); // Value Function Second Derivative

        k = new Vector[N];
        K = new Matrix[N];

        // Backwards Pass: Compute control update.
        for (int i = N-1; i >= 0; i--) {
            Vector control = currentControls[i];
            Vector state = currentTrajectory[i];

            // Compute most-used derivatives and their transposes once, for convenience and performance.
            Matrix dfdx = model.dFdX(state, control, dt);
            Matrix dfdxT = dfdx.transposed();
            Matrix dfdu = model.dFdU(state, control, dt);
            Matrix dfduT = dfdu.transposed();

            // Compute additional derivatives of "Q," the "Quality" of a control trajectory change.
            // This is then used as a second-order taylor expansion for trajectory improvement.
            // Uses Cost Function Combined with system dynamics
            Vector Qx = dCdX(state, referenceSignal.target(), dt).added(dfdxT.multiplied(vx));
            Vector Qu = dCdU(control, dt).added(dfduT.multiplied(vx));

            // Computing the second derivatives involves tensor multiplications, which the current libraries cannot handle.
            // However, these are not strictly necessary for convergence.
            // TODO: Implement tensor multiplications manually, or switch libraries.
            Matrix Qxx = dCdX2(dt).added(dfdxT.multiplied(vxx).multiplied(dfdx));//.added(model.VdF2dXdX(state, control, vx, dt));
            Matrix Qux = dfduT.multiplied(vxx).multiplied(dfdx);//.added(model.VdF2dXdU(state, control, vx, dt));
            GeneralMatrix Quu = (GeneralMatrix) dCdU2(dt).added(dfduT.multiplied(vxx).multiplied(dfdu));//.added(model.VdF2dUdU(state,control,vx, dt));

            // For better convergence, a Levenberg–Marquardt heuristic is used. However, this involves adjusting the eigenvectors.
            // The library built-in to the SDK does not support this, however I prefer it's API to alternatives.
            // Thus, I'm using the built-in for most operations, and delegating to EJML (https://ejml.org) for additional operations.
            DMatrixRMaj Quu2 = new DMatrixRMaj(this.numControls, this.numControls, true, Quu.getData());

            EigenDecomposition_F64<DMatrixRMaj> eigs = DecompositionFactory_DDRM.eig(true);
            eigs.decompose(Quu2);

            DMatrixRMaj EVecs = EigenOps_DDRM.createMatrixV(eigs);
            DMatrixRMaj EVals = EigenOps_DDRM.createMatrixD(eigs);

            for (int eign = 0; eign < EVals.numRows; eign++) {
                if (EVals.get(eign, eign) < 0) {
                    EVals.set(eign, eign, 0);
                }
                EVals.add(eign, eign, lambda);
                EVals.set(eign, eign, 1/EVals.get(eign, eign));
            }

            DMatrixRMaj Quu2_inv = new DMatrixRMaj(this.numControls, this.numControls);
            DMatrixRMaj temp = new DMatrixRMaj(this.numControls, this.numControls);
            CommonOps_DDRM.mult(EVecs, EVals, temp);
            CommonOps_DDRM.invert(EVecs);
            CommonOps_DDRM.mult(temp, EVecs, Quu2_inv);

            Matrix Quu_inv = new GeneralMatrix(this.numControls, this.numControls, Quu2_inv.getData());

            // Compute feedback and feedforward control changes.
            k[i] = Quu_inv.multiplied(Qu).multiplied(-1);
            K[i] = Quu_inv.multiplied(Qux).multiplied(-1);

            // Adjust second derivative of value function with current timestep.
            vx = Qx.subtracted(K[i].transposed().multiplied(k[i]));
            vxx = Qxx.subtracted(K[i].transposed().multiplied(K[i]));
        }


        Vector oldstate = currentTrajectory[0];
        currentTrajectory[0] = currentState;

        // Forwards pass: Update state
        for (int i = 0; i < N; i++) {
            // Compute updated control trajectory
            currentControls[i] = currentControls[i].added(k[i]);
            currentControls[i].add(K[i].multiplied(currentTrajectory[i].subtracted(oldstate)));

            if (i != N-1) {

                // Update state trajectory
                oldstate = currentTrajectory[i + 1];
                currentTrajectory[i + 1] = model.stateTransitionFunction(currentTrajectory[i], currentControls[i], dt);

                // If the control has gone past it's target, and it's final state is not static, then the horizon time needs to be shortened.
                // TODO: Make this better at not-drivetrains
                Vector targetPosition = new Vector(referenceSignal.target().get(0), referenceSignal.target().get(1));
                Vector positionError = targetPosition.subtracted(new Vector(currentTrajectory[i].get(0), currentTrajectory[i].get(1)));
                Vector nextPositionError = targetPosition.subtracted(new Vector(currentTrajectory[i+1].get(0), currentTrajectory[i+1].get(1)));

                // If the dotproduct of the position error is zero, then it has gone past the target.
                if (Math.signum(positionError.dotProduct(nextPositionError)) <=0 && (referenceSignal.target().get(3) != 0 || referenceSignal.target().get(4) != 0 )) {

                    // Update the trajectory with a new horizon time.
                    double newhorizon = (i+1)*dt;
                    double newdt = newhorizon/N;
                    BaseOpMode.addData("Setting horizon to", newhorizon);

                    for (int j = 0; j < N; j++) {
                        double t = newdt * j;
                        currentControls[j] = getInterpolatedU(t, newhorizon);
                        currentTrajectory[j] = getInterpolatedX(t, newhorizon);
                        k[j] = getInterpolatedk(t, newhorizon);
                        K[j] = getInterpolatedK(t, newhorizon);
                    }

                    this.dt = newdt;
                    this.horizon = newhorizon;
                }
            }
        }

    }

    /**
     * Used for Model Precitive Control. Set trajectory information based off current position.
     * @param currentPos    Position at current timestep.
     * @param currentN      Current Timestep
     */
    protected void stepForwardHorizon(Vector currentPos, int currentN) {

        if (currentN == 0) return;

        // As we're no longer necessarily on the trajectory, we should reset lambda.
        lambda = 1;

        // Update trajectory variables
        for (int i = 0; i < N - currentN; i++) {
            this.currentControls[i] = this.currentControls[currentN + i];
            this.currentTrajectory[i] = this.currentTrajectory[currentN + i];
            this.k[i] = this.k[currentN + i];
            this.K[i] = this.K[currentN + i];
        }

        // Update controls for future state
        for (int i = currentN; i < N; i++) {
            this.k[i] = this.k[i-1];
            this.K[i] = this.K[i-1];
            this.currentControls[i] = currentControls[N-1].added(k[i]);
        }

        Vector oldstate = currentTrajectory[0];
        currentTrajectory[0] = currentPos;

        // Simulate new trajectory to get approximation with new controls.
        for (int i = 0; i < N; i++ ) {
            currentControls[i] = currentControls[i].added(k[i]);
            currentControls[i].add(K[i].multiplied(currentTrajectory[i].subtracted(oldstate)));

            if (i != N-1) {
                oldstate = currentTrajectory[i + 1];
                currentTrajectory[i + 1] = model.stateTransitionFunction(currentTrajectory[i], currentControls[i], dt);
            }
        }
    }


    /**
     * Instead of compiling from scratch, it may be faster to load variables from an array.
     * @param x     State Trajectory
     * @param u     Control Trajectory
     * @param k     Control Feedforward
     * @param K     Feedback Trajectory
     */
    public void loadFromArray(Vector[] x, Vector[] u, Vector[] k, Matrix[] K) {
        this.currentTrajectory = x;
        this.currentControls = u;
        this.k = k;
        this.K = K;
    }

    /**
     * Iterate updateControls using Levenberg–Marquardt Heuristics to update lambda.
     * @param maxIter   Maximum number of iterations to do. Prevents getting stuck in a loop.
     * @param start     Start position to simulate from.
     */
    public void iterate(int maxIter, Vector start) {

        // Get initial setup.
        double oldcost = getTotalCost();
        BaseOpMode.addData("MPC: Initial Cost", oldcost);

        Vector[] xold = currentTrajectory.clone();
        Vector[] uold = currentControls.clone();
        Vector[] kold = k.clone();
        Matrix[] Kold = K.clone();
        double cost;

        for (int iter = 0; (iter < maxIter) || (maxIter == 0); iter++) {

            // Update trajectory
            updateControls(start);
            cost = getTotalCost();


            // Telemetry
            BaseOpMode.addData("MPC: Iteration", iter);
            BaseOpMode.addData("MPC: lambda", lambda);

            // If it's an improvement, save it.
            if (cost < oldcost ) {

                // Telemetry
                BaseOpMode.addData("MPC: Cost", cost);
                BaseOpMode.addData("MPC: Old Cost", oldcost);
                BaseOpMode.addData("MPC: Improvement", oldcost - cost);

                BaseOpMode.addData("MPC: Landing X", currentTrajectory[N-1].get(0));
                BaseOpMode.addData("MPC: Landing Y", currentTrajectory[N-1].get(1));
                BaseOpMode.addData("MPC: Landing H", currentTrajectory[N-1].get(2));

                lambda /= lr;
                xold = currentTrajectory.clone();
                uold = currentControls.clone();
                kold = k.clone();
                Kold = K.clone();

                // If the threshold has been passed, then stop iterating.
                if (Math.abs(cost-oldcost)/cost < threshold) {
                    break;
                }
                oldcost = cost;
            }
            else {
                lambda *= lr;

                currentTrajectory = xold.clone();
                currentControls = uold.clone();
                k = kold.clone();
                K = Kold.clone();
                if (lambda > lambda_max) {
                    break;
                }
            }

            // This is blocking code, so the telemetry won't automatically be updating.
            BaseOpMode.updateTelemetry();

        }

    }
}
