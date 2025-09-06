package org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Vector;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class BlockDetection {
    //public Limelight3A limelight;

    public Vector centerBlue;
    public Vector centerYellow;
    public static double scale = 0.0015;
    public static double scaleX = -1;
    public static double scaleY = 1;

    public static double scaleTheta = -0.01;

    public double angleBlue;
    public double angleYellow;

    public VisionPortal vision;
    private WebcamName webcam;

    public IntoTheDeepVisionProcessor processor;

    public double[] displacement;




    public BlockDetection() {
        //limelight = BaseOpMode.hardware.get(Limelight3A.class, Hardware.limelight);

        //limelight.stop();

        //limelight.setPollRateHz(1000);
        //limelight.pipelineSwitch(0);

        //limelight.start();

        webcam = BaseOpMode.hardware.get(WebcamName.class, Hardware.blockCam );

        processor = new IntoTheDeepVisionProcessor();

        vision = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();


    }

    public void update() {
        /*LLResult result = limelight.getLatestResult();

        if (result !=null) {
            double[] limelightOutput = result.getPythonOutput();
            this.centerBlue = new Vector(scaleX*(limelightOutput[2]-320), scaleY*(limelightOutput[3]-240)).multiplied(scale);
            this.centerYellow = new Vector(limelightOutput[4], limelightOutput[5]).multiplied(scale);
            this.angleBlue = scaleTheta*limelightOutput[6];
            this.angleYellow = scaleTheta*limelightOutput[7];

        }

         */

        displacement = processor.getDisplacement(true);


    }
}
