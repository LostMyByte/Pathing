package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Learning.PIDTuningDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Vector;

import java.io.File;
import java.io.FileWriter;

@TeleOp(name = "PID Tuning")
public class PIDTuningTeleOP extends BaseOpMode {
    Vector movement = new Vector(0, 10);

    private double start = -1;


    private PIDTuningDriveTrain drive;

    private File file;
    private FileWriter writer;

    public static String path = "/storage/emulated/0/data.txt";


    @Override
    public void externalInit() {
        drive = new PIDTuningDriveTrain(0, 50);
        file = new File(path);
        try {
            if (!file.exists()) {
                file.createNewFile();
                telemetry.addData("Made File", path);
            }
            else {
                file.delete();
                file.createNewFile();
            }
        }
        catch (java.io.IOException e) {
            telemetry.addData("Error:", e);
        }
        try {
            writer = new FileWriter(file);
        }
        catch (java.io.IOException e) {
            telemetry.addData("Error:", e);
        }


    }

    @Override
    public void externalLoop() {
        drive.train(new Vector(0.1, 0.0, 0.0), 4, 10);
        Vector result = drive.bestAgent;
        addData("kP", result.get(0));
        addData("kI", result.get(1));
        addData("kD", result.get(2));


    }
}


