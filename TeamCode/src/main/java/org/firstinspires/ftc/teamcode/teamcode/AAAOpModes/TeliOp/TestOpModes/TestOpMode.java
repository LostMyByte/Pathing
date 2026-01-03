// Primary Author: Caroline Oringer

package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions.servoTest;

import android.graphics.Color;
import android.widget.BaseExpandableListAdapter;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {

  /*  NormalizedColorSensor sensor;
    final float[] hsvValues = new float[3];*/

    TeliOpDrivetrain drive;
    Location loc;
    IntakeMagazine intake;
    Shooter shooter;
    boolean firstLoop = true;
    ElapsedTime timer;
    Servos.Turret servo;
    Servos.Turret2 servo2;
    @Override
    public void externalInit() {
        //sensor = hardwareMap.get(NormalizedColorSensor.class, "Color");
        /*
        drive = new TeliOpDrivetrain(hardwareMap, 0);
        loc = new Location(0,300,0);
        intake = new IntakeMagazine(hardwareMap);
        shooter = new Shooter(hardwareMap, Constants.Team.RED);
        intake.setState(IntakeMagazine.IntakeMagazineStates.IDLE);
        shooter.setState(Shooter.ShooterStates.ACTIVE);*/
        servo = new Servos.Turret();
        servo2 = new Servos.Turret2();

    }

    @Override
    public void externalLoop() {
        if (timer == null) timer = new ElapsedTime();
       /* NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        */
        /*
        drive.updateOdo(loc.getPosX(), loc.getPosY(), loc.getPosH());
        shooter.recieveOdoInputs(loc.getPosX(), loc.getPosY(), loc.getPosH(), new Vector(0,0,0), loc.getVelH());
        BaseOpMode.addData("headingVelocity", loc.getVelH());

        if (driver1.cross.isTapped()){
            firstLoop = true;
        }

        if (driver1.cross.isToggled()){
            drive.driveDumb(0,0,1,firstLoop);
            firstLoop = false;
        } else {
            drive.driveDumb(0,Math.PI,1,firstLoop);
            //if (timer.seconds() > 5) drive.dumbDriveToPos(-120,120,0.2);
            firstLoop = false;
        }
        */
        //servo.setPositionInterpolated(servoTest);
        servo2.setPositionInterpolated(servoTest);
    }
}
