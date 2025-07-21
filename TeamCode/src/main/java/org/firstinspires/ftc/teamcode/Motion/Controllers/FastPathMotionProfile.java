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
        this.startPosition = targetPath.getPosition(0);
        this.time = 0;
        this.scalar = speed;
        this.speed = speed;
    }

    @Override
    protected void update() {
        Vector velocityVector = targetPath.getVelocity(time);
        Vector accelerationVector = targetPath.getAcceleration(time);
        scalar = FixedDriveTrain.getPowerScalar(accelerationVector.normalized()) * DriveConfig.DriveWheels.driveAcceleration /accelerationVector.magnitude() * speed;

        this.data = velocityVector.multiplied(scalar);
        BaseOpMode.addData("Target Velocity X", data.getData()[0]);
        BaseOpMode.addData("Target Velocity Y", data.getData()[1]);
        BaseOpMode.addData("Target Velocity H", data.getData()[2]);
        time += deltaTime * scalar;
    }

    @Override
    public Vector getGradient() {
        return targetPath.getAcceleration(time).multiplied(scalar);
    }

    @Override
    public Vector getIntegralVector() {
        return targetPath.getPosition(time).subtracted(startPosition);
    }

    @Override
    public void resetIntegral() {
        this.startPosition = targetPath.getPosition(time);
    }
}
