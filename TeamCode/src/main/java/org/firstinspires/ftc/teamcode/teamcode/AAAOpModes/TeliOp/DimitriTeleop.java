package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter.ShooterStates.ACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter.ShooterStates.NOTACTIVE;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class DimitriTeleop extends BaseOpMode{
    Shooter shooter;
    IntakeMagazine intake;
    TeliOpDrivetrain drive;
    boolean shooterActive = true;
    boolean wasIntaking = false;

    Location loc;

    @Override
    public void externalInit() {
        shooter = new Shooter(hardwareMap, 0, Constants.team);
        intake = new IntakeMagazine(hardwareMap);
        drive = new TeliOpDrivetrain(hardwareMap,0);
        loc = new Location(0, 0,0);
    }

    @Override
    public void externalLoop() {
        if (driver1.rightStick.isPressed()){
            drive.PIDdrive(driver1.leftStick.Y(), -driver1.rightStick.X(), 0.3);
        } else {
            drive.PIDdrive(driver1.leftStick.Y(), -driver1.rightStick.X(), 1);
        }
        shooter.recieveOdoInputs(loc.getPosX(), loc.getPosY(), loc.getPosH(), loc.getTranslationalVelocity());
        intake.setIndexMode(driver1.options.isToggled());

        if (shooterActive){
            shooter.setState(ACTIVE);
        } else {
            shooter.setState(NOTACTIVE);
        }

        if (driver1.dpad_up.isTapped()){
            shooterActive = !shooterActive;
        }

        if (driver1.leftTrigger.isPressed()){
            intake.setState(IntakeMagazine.IntakeMagazineStates.INTAKEREAR);
            wasIntaking = true;
        } else if (driver1.rightTrigger.isPressed()){
            intake.setState(IntakeMagazine.IntakeMagazineStates.INTAKEFRONT);
            wasIntaking = true;
        } else if (driver1.rightBumper.isPressed()){
            intake.setState(IntakeMagazine.IntakeMagazineStates.CLEAR);
            wasIntaking = true;
        } else if (wasIntaking){
            intake.setState(IntakeMagazine.IntakeMagazineStates.IDLE);
            wasIntaking = false;
        }

        if (intake.getState() == IntakeMagazine.IntakeMagazineStates.IDLE && driver1.cross.isTapped()){
            if (!intake.indexMode) {
                intake.setState(IntakeMagazine.IntakeMagazineStates.SHOOTING);
            } else {
                intake.setState(IntakeMagazine.IntakeMagazineStates.LOADANDSHOOTUNINDEXED);
            }
        } else if (intake.indexMode && intake.getState() == IntakeMagazine.IntakeMagazineStates.IDLE && driver1.square.isTapped()){
            intake.setState(IntakeMagazine.IntakeMagazineStates.LOADPURPLE);
        } else if (intake.indexMode && intake.getState() == IntakeMagazine.IntakeMagazineStates.IDLE && driver1.triangle.isTapped()){
            intake.setState(IntakeMagazine.IntakeMagazineStates.LOADGREEN);
        }
    }
}
