// Primary Author: Kieran Mattingly
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

    /**
     * Follows a given path.
     * Here for reference and potential future use, however the MPC path framework is more versatile and faster.
     * @param path  Path to follow
     * @param speed Speed to follow at
     */
    @Deprecated
    public void followPath(Path path, double speed) {
        if (activePath != path) {
            activePath = path;
            if (path != null) {
                profile = new MotionProfile(activePath, speed);
                correctionSignal = new PID(profile, loc, DriveWheels.driveConstants);
            }
        }
    }

    /**
     * Move according to a power vector.
     * @param target    Target power vector.
     */
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
