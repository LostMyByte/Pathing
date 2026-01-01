// Primary Author: Caroline Oringer

package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

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

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;

@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {

  /*  NormalizedColorSensor sensor;
    final float[] hsvValues = new float[3];*/

    TeliOpDrivetrain drive;
    Location loc;
    boolean firstLoop = true;
    @Override
    public void externalInit() {
        //sensor = hardwareMap.get(NormalizedColorSensor.class, "Color");
        drive = new TeliOpDrivetrain(hardwareMap, 0);
        loc = new Location(0,0,0);
    }

    @Override
    public void externalLoop() {
       /* NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        */
        drive.updateOdo(loc.getPosX(), loc.getPosY(), loc.getPosH());

        if (driver1.cross.isTapped()){
            firstLoop = true;
        }

        if (!driver1.cross.isToggled()){
            drive.driveDumb(0,0,1, firstLoop);
            firstLoop = false;
        } else {
            drive.driveDumb(0,Math.PI,1,firstLoop);
            firstLoop = false;
        }

    }
}
