package org.firstinspires.ftc.teamcode.Utilities.HardwareDevices;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;

import java.util.ArrayList;

public class MotorEncoder {

    public static ArrayList<MotorEncoder> encoders = new ArrayList<>();

    DcMotorImplEx encoder;
    double position = 0;
    double offset = 0;
    double velocity = 0;
    double tickMultiplier = 1;

    /**
     * Gets new encoder values for all encoders
     */

    public static void updateEncoders(){
        for(MotorEncoder e : encoders){
            e.update();
        }
    }

    public static void resetEncoderList(){
        encoders.clear();
    }

    /**
     * @return - returns the encoder value
     */
    public double getPosition(){
        return position + offset;
    }

    /**
     * @return - returns encoder velocity
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * returns the encoder and offset to 0
     */
    public void resetEncoder(){
        position = 0;
        offset = 0;

        encoder.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public MotorEncoder(String name, double tickMultiplier){
        encoders.add(this);
        encoder = (DcMotorImplEx) BaseOpMode.hardware.dcMotor.get(name);

        this.tickMultiplier = tickMultiplier;

        resetEncoder();
    }

    public MotorEncoder(String name){
        this(name, 1);
    }

    private void update(){
        position = encoder.getCurrentPosition() * tickMultiplier;
        velocity = encoder.getVelocity() * tickMultiplier;
    }

    public void setOffset(double offset){
        this.offset = offset;
    }

    public void setTickMultiplier(double m){
        tickMultiplier = m;
    }
}
