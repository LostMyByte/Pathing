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
public class OdoPodData extends Signal {

    public GoBildaPinpointDriver odoPods;

    public static double xOffset = 10;
    public static double yOffset = -90;
    public static double alpha = 1;

    double oldAngle;

    double driveVelocity = 0;

    private void initialize() {

        odoPods = BaseOpMode.getHardwareMap().get(GoBildaPinpointDriver.class, Hardware.odoWheels);
        odoPods.setOffsets(xOffset,yOffset);
        odoPods.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odoPods.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odoPods.recalibrateIMU();
    }

    public OdoPodData(double startX, double startY, double startH) {
        super(3);
        initialize();
        this.data = new Vector(startX,startY,startH);
        this.oldAngle = startH;
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, startX, startY, AngleUnit.RADIANS, startH));
    }

    Vector velocity = new Vector(0, 0, 0);
    @Override
    public void update() {
        odoPods.update();

        velocity = new Vector(odoPods.getVelX(DistanceUnit.CM), odoPods.getVelY(DistanceUnit.CM),odoPods.getHeadingVelocity());

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
    public Vector getGradient() {
        return new Vector(
                odoPods.getVelX(DistanceUnit.CM),
                odoPods.getVelY(DistanceUnit.CM),
                odoPods.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );
    }


    @Override
    public void telemetry() {

        BaseOpMode.addData("Odo Velocity X", getDerivatives()[0]);
        BaseOpMode.addData("Odo Velocity Y", getDerivatives()[1]);
        BaseOpMode.addData("Odo Velocity H", getDerivatives()[2]);

        BaseOpMode.addData("Odo Position X", data.get(0));
        BaseOpMode.addData("Odo Position Y", data.get(1));
        BaseOpMode.addData("Odo Position H", data.get(2));
    }

    public void setPosition(Vector pos) {
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, pos.get(0), pos.get(1), AngleUnit.RADIANS, pos.get(2)));
    }

    public void setPosition(double x, double y, double h) {
        odoPods.setPosition(new Pose2D(DistanceUnit.CM, x, y, AngleUnit.RADIANS, h));
    }



    public void updateOffsets() {
        odoPods.setOffsets(xOffset, yOffset);
    }

}
