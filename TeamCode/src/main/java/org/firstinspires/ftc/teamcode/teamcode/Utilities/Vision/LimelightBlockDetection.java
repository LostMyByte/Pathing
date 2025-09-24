package org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;

@Config
public class LimelightBlockDetection extends Subsystem {
    public double[] yellowDisplacement = new double[2];
    public double[] redDisplacement = new double[2];
    public double[] blueDisplacement = new double[2];

    public double yMin = 15.5;

    private Limelight3A limelight;

    public static double brightness = 1.0;

    private Servo headlight; // Yes, this is how it works. No, I don't know why. But you program headlights as servos.

    public LimelightBlockDetection() {
        limelight = BaseOpMode.hardware.get(Limelight3A.class, Hardware.limelight);

       // headlight = new Servo(Hardware.headlight);
       // headlight.setPosition(brightness); // Turn on the headlight

        limelight.getStatus();

        limelight.start();
    }

    @Override
    public void update() {

        limelight.updatePythonInputs(new double[]{yMin, 0, 0});
        headlight.setPosition(brightness); // Turn on the headlight

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] output = result.getPythonOutput();
            if (output[1] != 0 && output[1] < 30) {

                yellowDisplacement[0] = output[0];
                yellowDisplacement[1] = output[1];

                redDisplacement[0] = output[2];
                redDisplacement[1] = output[3];

                blueDisplacement[0] = output[4];
                blueDisplacement[1] = output[5];

                BaseOpMode.addData("Yellow X", output[0]);
                BaseOpMode.addData("Yellow Y", output[1]);
            }
        }
    }
    @Override
    public void updateSensors() {

    }
}
