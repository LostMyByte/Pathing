package org.firstinspires.ftc.teamcode.Motion.Localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Signal;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

@Config
public class Location extends Signal {
    // A Velocity-based Location class
    // Uses Velocity as that is the level upon which the Motion Profile/PIDs work
    // Has convenience methods for x, y, etc. access. I'm not trying to give Dylan a headache.

    public GoBildaPinpointDriver odoPods;

    public static double xOffset = 0;
    public static double yOffset = 21;


    public static double alpha = 1;

    double oldAngle;

    private void initialize() {

        odoPods = BaseOpMode.getHardwareMap().get(GoBildaPinpointDriver.class, Hardware.odoWheels);
        odoPods.setOffsets(xOffset,yOffset);
        odoPods.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odoPods.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odoPods.recalibrateIMU();
    }

    public Location(double startX, double startY, double startH) {
        super(3);
        initialize();
        this.data = new Vector(0,0,0);
        this.oldAngle = startH;
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, startX, startY, AngleUnit.RADIANS, startH));
    }

    public Location(Vector startState) {
        super(3);
        initialize();
        this.data = Vector.length(3);
        this.oldAngle = startState.get(2);
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, startState.get(0), startState.get(1), AngleUnit.RADIANS, startState.get(2)));
    }

    public Vector getPosition() {
        return getIntegralVector();
    }

    Vector oldData = new Vector(0, 0, 0);
    @Override
    public void update() {
        odoPods.update();

        Vector newData = new Vector(odoPods.getVelX(DistanceUnit.CM), odoPods.getVelY(DistanceUnit.CM),odoPods.getHeadingVelocity());

        this.data.add(newData.subtracted(this.data).multiplied(alpha));
        this.oldData = newData;

        Pose2D pose = odoPods.getPosition();
        double angle = pose.getHeading(AngleUnit.RADIANS);
        while (angle - oldAngle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle - oldAngle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        oldAngle = angle;
    }


    @Override
    public Vector getIntegralVector() {
        Pose2D pose = odoPods.getPosition();
        return new Vector(pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), oldAngle);
    }

    // Don't worry about the integration because we have a better source
    @Override
    protected void addIntegral() {}

    @Override
    public void telemetry() {

        Vector pos = getPosition();
        double angle = pos.get(2);


        BaseOpMode.addData("Velocity X", data.getData()[0]);
        BaseOpMode.addData("Velocity Y", data.getData()[1]);
        BaseOpMode.addData("Velocity H", data.getData()[2]);

        BaseOpMode.addData("Position X", pos.get(0));
        BaseOpMode.addData("Position Y", pos.get(1));
        BaseOpMode.addData("Position H", pos.get(2));
    }

    public void setPosition(Vector pos) {
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, pos.get(0), pos.get(1), AngleUnit.RADIANS, pos.get(2)));
    }

    public Vector getPositionForTankDrive() {
        Vector pos = getPosition();
        double angle = pos.get(2);
        double v = new Vector(Math.cos(-angle), Math.sin(-angle)).dotProduct(new Vector(data.get(1), data.get(0)));
        BaseOpMode.addData("Velocity Drive", v);
        return new Vector(new double[] {
                pos.get(0),
                pos.get(1),
                pos.get(2),
                v,
                data.get(2)
        });
    }

    public void updateOffsets() {
        odoPods.setOffsets(xOffset, yOffset);
    }

}
