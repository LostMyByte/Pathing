package org.firstinspires.ftc.teamcode.Motion.Localization;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Signal;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;

public class Location extends Signal {
    // A Velocity-based Location class
    // Uses Velocity as that is the level upon which the Motion Profile/PIDs work
    // Has convenience methods for x, y, etc. access. I'm not trying to give Dylan a headache.

    GoBildaPinpointDriver odoPods;
    IMU imu;

    private void initialize() {
        this.imu = BaseOpMode.getHardwareMap().get(IMU.class, Hardware.imu);
        odoPods = BaseOpMode.getHardwareMap().get(GoBildaPinpointDriver.class, Hardware.odoWheels);
        odoPods.setOffsets(-175,41.25);
        odoPods.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odoPods.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odoPods.recalibrateIMU();
    }

    public Location(double startX, double startY, double startH) {
        super(3);
        initialize();
        this.data = new Vector(0,0,0);
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, startX, startY, AngleUnit.RADIANS, startH));
    }

    public Location(Vector startState) {
        super(3);
        initialize();
        this.data = Vector.length(3);
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, startState.get(0), startState.get(1), AngleUnit.RADIANS, startState.get(2)));
    }

    public Vector getPosition() {
        return getIntegralVector();
    }

    @Override
    public void update() {
        odoPods.update();
        Pose2D pose = odoPods.getVelocity();
        this.data = new Vector(pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public Vector getIntegralVector() {
        Pose2D pose = odoPods.getPosition();
        return new Vector(pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.RADIANS));
    }

    // Don't worry about the integration because we have a better source
    @Override
    protected void addIntegral() {}

    @Override
    public void telemetry() {
        BaseOpMode.addData("Velocity X", data.getData()[0]);
        BaseOpMode.addData("Velocity Y", data.getData()[1]);
        BaseOpMode.addData("Velocity H", data.getData()[2]);

        BaseOpMode.addData("Position X", getIntegralVector().get(0));
        BaseOpMode.addData("Position Y", getIntegralVector().get(1));
        BaseOpMode.addData("Position H", getIntegralVector().get(2));
    }
}
