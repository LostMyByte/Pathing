// Primary Author: Caroline Oringer

package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import android.widget.BaseExpandableListAdapter;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {

    AnalogInput pin0;

    @Override
    public void externalInit() {
        pin0 = hardwareMap.analogInput.get("pin0");
    }

    @Override
    public void externalLoop() {

        BaseOpMode.addData("pin0", pin0.getVoltage());

    }
}
