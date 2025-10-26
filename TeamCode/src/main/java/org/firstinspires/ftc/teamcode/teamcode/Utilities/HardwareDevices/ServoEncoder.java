// Primary Author: Past Team Members, Mixed
package org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

public class ServoEncoder {

    AnalogInput servo;


    public ServoEncoder(String mapName){
        servo = BaseOpMode.getHardwareMap().get(AnalogInput.class, mapName);
    }

    public double getPositionDegrees(){
        return servo.getVoltage() / 3.3 * 360;
    }

    public double getPositionRadians(){
        return servo.getVoltage() / 3.3 * 6.28;
    }

}
