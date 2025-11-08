// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Localization;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

/**
 * A Position-based Location class. (I.e position is the data, absement the integral, and velocity the derivative)
 * Has convenience methods for x, y, etc. access. I'm not trying to give Dylan a headache.
 */
@Config
public class Location extends Signal {

    public GoBildaPinpointDriver odoPods;

    public static double xOffset = 0;
    public static double yOffset = 21;
    public static double alpha = 1;

    double oldAngle;

    double driveVelocity = 0;

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
        this.data = new Vector(startX,startY,startH);
        this.oldAngle = startH;
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, startX, startY, AngleUnit.RADIANS, startH));
    }

    public Location(Vector startState) {
        super(3);
        initialize();
        this.data = startState;
        this.oldAngle = startState.get(2);
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, startState.get(0), startState.get(1), AngleUnit.RADIANS, startState.get(2)));
    }

    public Location() {
        super(3);
        initialize();
        Pose2D pose = odoPods.getPosition();
        this.data = new Vector(pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.RADIANS))
        this.oldAngle = data.get(2);
    }

    public Vector getPosition() {
        return getDataVector();
    }



    Vector velocity = new Vector(0, 0, 0);
    @Override
    public void update() {
        odoPods.update();

        Vector newVelocity = new Vector(odoPods.getVelX(DistanceUnit.CM), odoPods.getVelY(DistanceUnit.CM),odoPods.getHeadingVelocity());

        this.velocity.add(newVelocity.subtracted(this.velocity).multiplied(alpha));
        this.velocity = newVelocity;

        Pose2D pose = odoPods.getPosition();
        double angle = pose.getHeading(AngleUnit.RADIANS);
        while (angle - oldAngle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle - oldAngle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        oldAngle = angle;

        this.data = new Vector(pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), oldAngle);

        driveVelocity = new Vector(Math.cos(-angle), Math.sin(-angle)).dotProduct(new Vector(velocity.get(1), velocity.get(0)));
        BaseOpMode.addData("Velocity Drive", driveVelocity);
    }

    @Override
    public void telemetry() {

        Vector pos = getPosition();

        BaseOpMode.addData("Velocity X", velocity.getData()[0]);
        BaseOpMode.addData("Velocity Y", velocity.getData()[1]);
        BaseOpMode.addData("Velocity H", velocity.getData()[2]);

        BaseOpMode.addData("Position X", pos.get(0));
        BaseOpMode.addData("Position Y", pos.get(1));
        BaseOpMode.addData("Position H", pos.get(2));
    }

    public void setPosition(Vector pos) {
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, pos.get(0), pos.get(1), AngleUnit.RADIANS, pos.get(2)));
    }

    public void setPosition(double x, double y, double h) {
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, x, y, AngleUnit.RADIANS, h));
    }

    public Vector getPositionForTankDrive() {
        Vector pos = getPosition();

        return new Vector(new double[] {
                pos.get(0),
                pos.get(1),
                pos.get(2),
                driveVelocity,
                data.get(2)
        });
    }

    public void updateOffsets() {
        odoPods.setOffsets(xOffset, yOffset);
    }

    public double getPosX() {
        return odoPods.getPosX(DistanceUnit.CM);
    }
    public double getPosY() {
        return odoPods.getPosY(DistanceUnit.CM);
    }
    public double getPosH() {
        return odoPods.getHeading(AngleUnit.RADIANS);
    }
    public double getVelDrive() {
        return driveVelocity;
    }
    public double getVelH() {
        return odoPods.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }
    public Vector getTranslationalVelocity() {
        return new Vector(odoPods.getVelX(DistanceUnit.CM), odoPods.getVelY(DistanceUnit.CM));
    }

}
