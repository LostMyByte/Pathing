package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.DONOTHING;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter.ShooterStates.ACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter.ShooterStates.NOTACTIVE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

@TeleOp
public class DimitriTeleop extends BaseOpMode{
    Shooter shooter;
    IntakeMagazine intake;
    //0TeliOpDrivetrain drive;
    boolean shooterActive = true;
    boolean wasIntaking = false;

    Location loc;

    @Override
    public void externalInit() {
        Constants.team = Constants.Team.RED;
        shooter = new Shooter(hardwareMap, 0, Constants.Team.RED);
        shooter.setState(NOTACTIVE);
        intake = new IntakeMagazine(hardwareMap);
        intake.setState(DONOTHING);
        //drive = new TeliOpDrivetrain(hardwareMap,0);
        loc = new Location(183, 183,Math.PI/2);
        loc.doTelemetry = true;
    }

    @Override
    public void externalLoop() {

        //drive.PIDdrive(driver1.leftStick.Y(), -driver1.rightStick.X(), 1);

        shooter.recieveOdoInputs(loc.getPosX(), loc.getPosY(), loc.getPosH(), loc.getTranslationalVelocity());
        //shooter.panic = driver1.share.isToggled();

        if (!driver1.dpad_up.isToggled()){
            shooter.setState(ACTIVE);
        } else {
            shooter.setState(NOTACTIVE);
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
            intake.setState(IntakeMagazine.IntakeMagazineStates.SEATBALLS);
            wasIntaking = false;
        }

        if (intake.getState() == IntakeMagazine.IntakeMagazineStates.IDLE && driver1.cross.isTapped()) {
            intake.setState(IntakeMagazine.IntakeMagazineStates.SHOOTING);
        }


    }

}
