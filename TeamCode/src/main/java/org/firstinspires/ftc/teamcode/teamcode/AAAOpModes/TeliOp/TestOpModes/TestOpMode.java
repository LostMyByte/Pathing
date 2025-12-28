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
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;

@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {

  /*  NormalizedColorSensor sensor;
    final float[] hsvValues = new float[3];*/

    Servo servo;
    Servo servo2;
    @Override
    public void externalInit() {
        //sensor = hardwareMap.get(NormalizedColorSensor.class, "Color");
        servo = new Servo(Hardware.turret);
        servo2 = new Servo(Hardware.turret2);
    }

    @Override
    public void externalLoop() {
       /* NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        BaseOpMode.addData("hue",hsvValues[0]);*/
        servo.setPosition(DashPositions.servoTest);
        servo2.setPosition(DashPositions.servoTest);
    }
}
