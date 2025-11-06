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


    /**
     * Follows a given path.
     * Here for reference and potential future use, however the MPC path framework is more versatile and faster.
     * @param path  Path to follow
     * @param speed Speed to follow at
     * @param loc   Location signal
     */
    @Deprecated
    public void followPath(Path path, double speed, Location loc) {
        if (activePath != path) {
            activePath = path;
            if (path != null) {
                profile = new MotionProfile(activePath, speed);
                correctionSignal = new PID(profile, loc, DriveWheels.driveConstants);
            }
        }
    }

    /**
     * Follows a controller's signal
     */
    public void followController(Controller controller) {
        this.correctionSignal = controller;
    }

    /**
     * Move according to a power vector.
     * @param target    Target power vector.
     */
    public abstract void move(Vector target);

    public abstract void moveRaw(Vector target);

    @Override
    public void update() {
        if (correctionSignal != null) {
            moveRaw(correctionSignal.getCorrection());
            if (correctionSignal.targetPositionError().magnitude()<0.5) {
                Signal.signals.remove(profile);
                correctionSignal = null;
            }
        }
    }

    @Override
    public void updateSensors(){}
}
