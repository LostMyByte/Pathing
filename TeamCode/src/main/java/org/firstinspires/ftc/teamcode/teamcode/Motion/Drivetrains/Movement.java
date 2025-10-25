package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.Controller;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.MotionProfile;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.PID;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ReferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Paths.Path;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public abstract class Movement extends Subsystem {

    public Path activePath;
    public Controller correctionSignal;
    public ReferenceSignal profile;

    public Location loc;

    protected Motor[] driveWheels;

    public void followPath(Path path, double speed) {
        if (activePath != path) {
            activePath = path;
            if (path != null) {
                profile = new MotionProfile(activePath, speed);
                correctionSignal = new PID(profile, loc, DriveWheels.driveConstants);
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
