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
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hopper;

@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {

  /*  NormalizedColorSensor sensor;
    final float[] hsvValues = new float[3];*/
    TouchSensor breakBeam;
    @Override
    public void externalInit() {
        //sensor = hardwareMap.get(NormalizedColorSensor.class, "Color");
        breakBeam = hardwareMap.get(TouchSensor.class, "pin0");
    }

    @Override
    public void externalLoop() {
       /* NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        BaseOpMode.addData("hue",hsvValues[0]);*/
        telemetry.update();

        BaseOpMode.addData("beam broken",breakBeam.getValue());
    }
}
