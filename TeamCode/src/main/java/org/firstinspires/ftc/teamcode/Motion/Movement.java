package org.firstinspires.ftc.teamcode.Motion;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Controller;
import org.firstinspires.ftc.teamcode.Motion.Controllers.FastPathMotionProfile;
import org.firstinspires.ftc.teamcode.Motion.Controllers.PID;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Signal;
import org.firstinspires.ftc.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.Motion.Paths.Path;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;

public abstract class Movement extends Subsystem {

    public Path activePath;
    public Controller correctionSignal;
    public Signal profile;

    public Location loc;

    public void followPath(Path path, double speed) {
        if (activePath != path) {
            activePath = path;
            if (path != null) {
                profile = new FastPathMotionProfile(activePath, speed);
                correctionSignal = new PID(profile, loc, DriveConfig.DriveWheels.driveConstants);
            }
        }
    }

    public abstract void move(Vector target);

    @Override
    public void update() {
        if (activePath != null) {
            if (activePath.getTarget().subtracted(loc.getPosition()).magnitude()<0.5) {
                Signal.signals.remove(profile);
                activePath = null;
                move(new Vector(0,0,0));
            }
            else {
                move(correctionSignal.getCorrection());
            }
        }
    }

    @Override
    public void updateSensors(){}

    public Movement(Vector startState) {
        loc = new Location(startState);
    }
}
