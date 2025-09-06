package org.firstinspires.ftc.teamcode.teamcode.Subsystems;




import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.DiffyClaw.ClawDash.kI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;


public class DiffyClaw extends Subsystem{
    Servo claw;
    boolean aBoolean = true;

    ElapsedTime timer = new ElapsedTime();

    double lastError = 0;
    double integral = 0;
    @Config
    public static class ClawDash{
        public static double kP = 0;
        public static double kI=0;
        public static double kD=0;
        public static double visionDeadzone = 1;

    }
   public DiffyClaw(){
        //claw = BaseOpMode.hardware.get(Servo.class, Hardware.diffyClaw);
   }

    @Override
    public void update() {

    }

    @Override
    public void updateSensors() {

    }

    public void moveToPickUpSample(Double angleBlue){
        if(aBoolean){
            double targetAngle = 0.0;
            double error = targetAngle - angleBlue;
            integral += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();
            lastError = error;
            timer.reset();

            // Calculate PID output
            double output = (ClawDash.kP * error) + (kI * integral) + (ClawDash.kD * derivative);

            // Set CRServo power based on PID output (adjust scale if necessary)
            // Positive output should rotate towards the target angle, negative away
            double crServoPower = Range.clip(output, -1.0, 1.0);
            claw.setPower(crServoPower);}
    }

    public void onOrOff(){
        aBoolean=!aBoolean;
    }
}


