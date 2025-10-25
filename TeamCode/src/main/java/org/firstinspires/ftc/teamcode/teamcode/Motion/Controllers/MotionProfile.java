package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Paths.Path;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class MotionProfile extends ReferenceSignal {


    Path targetPath;
    double time;
    double speed;

    Vector startPosition;

    public MotionProfile(Path targetPath, double speed) {
        super(3);
        this.targetPath = targetPath;
        this.time = 0;
        this.speed = speed;
    }

    @Override
    protected void update() {
        this.data = targetPath.getVelocity(time).multiplied(speed);
        time += deltaTime * speed;
    }

    @Override
    public Vector getGradient() {
        return targetPath.getAcceleration(time).multiplied(speed);
    }

    @Override
    public Vector getIntegralVector() {
        return targetPath.getPosition(time);
    }

    @Override
    public void resetIntegral() {
        this.startPosition = targetPath.getPosition(time);
    }

    @Override
    public void telemetry() {
        BaseOpMode.addData("Target Velocity X", data.getData()[0]);
        BaseOpMode.addData("Target Velocity Y", data.getData()[1]);
        BaseOpMode.addData("Target Velocity H", data.getData()[2]);
        BaseOpMode.addData("Target Acceleration (mag)", this.targetPath.getAcceleration(time).multiplied(speed).magnitude());

        BaseOpMode.addData("Profile Time", time);

        BaseOpMode.addData("Target X", getIntegralVector().get(0));
        BaseOpMode.addData("Target Y", getIntegralVector().get(1));
        BaseOpMode.addData("Target H", getIntegralVector().get(2));
    }


    @Override
    public Vector predict(double time) {
        return targetPath.getVelocity(this.time + time * speed);
    }

    @Override
    public Vector target() {
        return targetPath.getTarget();
    }
}
