package org.firstinspires.ftc.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.Paths.Path;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;

public class FastPathMotionProfile extends Signal {


    Path targetPath;
    double time;
    double speed;
    double scalar;
    Vector startPosition;

    public FastPathMotionProfile(Path targetPath, double speed) {
        super(3);
        this.targetPath = targetPath;
        this.time = 0;
        this.scalar = speed;
        this.speed = speed;
    }

    @Override
    protected void update() {

        if (time > 1) time = 1;
        Vector velocityVector = targetPath.getVelocity(time);
        Vector accelerationVector = targetPath.getAcceleration(time);
        double maxAcceleration = FixedDriveTrain.getPowerScalar(accelerationVector.normalized()) * DriveConfig.DriveWheels.driveAcceleration;
        scalar = maxAcceleration/accelerationVector.magnitude() * speed;

        this.data = velocityVector.multiplied(scalar);
        if (this.data.magnitude() > DriveConfig.DriveWheels.maxVelocity) {
            scalar *= DriveConfig.DriveWheels.maxVelocity/this.data.magnitude();
            this.data = velocityVector.multiplied(scalar);
        };

        time += deltaTime * scalar;

    }

    @Override
    public Vector getGradient() {
        return targetPath.getAcceleration(time).multiplied(scalar);
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
        BaseOpMode.addData("Target Acceleration (mag)", this.targetPath.getAcceleration(time).multiplied(scalar).magnitude());

        BaseOpMode.addData("Profile Time", time);
        BaseOpMode.addData("Profile Scalar", scalar);

        BaseOpMode.addData("Target X", getIntegralVector().get(0));
        BaseOpMode.addData("Target Y", getIntegralVector().get(1));
        BaseOpMode.addData("Target H", getIntegralVector().get(2));
    }


}
