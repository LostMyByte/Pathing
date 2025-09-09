package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;

public class Shooter extends Subsystem{

    Motor shooter1;
    Motor shooter2;
    Motor turret;
    Servo hood;
    AnalogInput turretEncoder;
    public Shooter(HardwareMap hardwareMap){
        shooter1 = new Motor(Hardware.shooter1);
        shooter2 = new Motor(Hardware.shooter2);
        hood = new Servo(Hardware.hood);
        turret = new Motor(Hardware.turret);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
    }
    @Override
    public void updateSensors() {

    }

    @Override
    public void update() {

    }

}
