package org.firstinspires.ftc.teamcode.Motion;

import org.firstinspires.ftc.teamcode.Motion.Controllers.Controller;
import org.firstinspires.ftc.teamcode.Motion.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Motion.Controllers.PID;
import org.firstinspires.ftc.teamcode.Motion.Controllers.ReferenceSignal;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Signal;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.DriveWheel;
import org.firstinspires.ftc.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.Motion.Paths.Path;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

public abstract class Movement extends Subsystem {

    public Path activePath;
    public Controller correctionSignal;
    public ReferenceSignal profile;

    public Location loc;

    protected DriveWheel[] driveWheels;

    public void followPath(Path path, double speed) {
        if (activePath != path) {
            activePath = path;
            if (path != null) {
                profile = new MotionProfile(activePath, speed);
                correctionSignal = new PID(profile, loc, DriveConfig.DriveWheels.driveConstants);
            }
        }
    }

    public abstract void move(Vector target);


    public abstract void moveRaw(Vector motorPowers);

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
        driveWheels = new DriveWheel[]{
                new DriveWheel(Hardware.rightFront, new Vector(DriveConfig.DriveWheels.FR.x, DriveConfig.DriveWheels.FR.y, DriveConfig.DriveWheels.FR.h)),
                new DriveWheel(Hardware.leftFront,  new Vector(DriveConfig.DriveWheels.FL.x, DriveConfig.DriveWheels.FL.y, DriveConfig.DriveWheels.FL.h)),
                new DriveWheel(Hardware.rightBack,  new Vector(DriveConfig.DriveWheels.BR.x, DriveConfig.DriveWheels.BR.y, DriveConfig.DriveWheels.BR.h)),
                new DriveWheel(Hardware.leftBack,   new Vector(DriveConfig.DriveWheels.BL.x, DriveConfig.DriveWheels.BL.y, DriveConfig.DriveWheels.BL.h))};
    }
}
