package org.firstinspires.ftc.teamcode.Utilities.HardwareDevives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;

import java.util.ArrayList;

public class Motor {

    public static ArrayList<Motor> motors = new ArrayList<>();
    private final DcMotorImplEx motor;
    public MotorEncoder encoder;

    public String motorName;
    private final double currentOverloadBuffer = 300;
    private final double powerScalar;
    private final ElapsedTime currentOverloadTimer = new ElapsedTime();
    private final ArrayList<Motor> pairedMotors = new ArrayList<>();
    private double power = 0;

    /**
     * commands all the motor powers to what they have been set
     */
    public static void commandPowers() {
        for (Motor m : motors) {
            m.commandPower();
        }
    }

    public static void resetMotorList() {
        motors.clear();
    }

    /**
     * sets all motor powers to zero
     */
    public static void stopMotors() {
        for (Motor m : motors) {
            m.setPower(0);
            m.commandPower();
        }
    }

    /**
     * Sets the power to be commanded
     *
     * @param p - power set to motor from -1 to 1
     */
    public void setPower(double p) {
        power = p;
        for (Motor m : pairedMotors) {
            m.setPower(p);
        }

//        BaseOpMode.addData("Motor " + motorName + " power set", p);

    }

    /**
     * returns the current draw in milli-amps
     */
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.MILLIAMPS);
    }

    /**
     * @param name           - hardware device name
     * @param reversed       - whether the motor should run backwards
     * @param encoder        - if the motor has an attached encoder
     * @param brake          - if the motor should brake when power = 0
     * @param maximumCurrent - maximum current in milli-amps before the motor will disable
     * @param powerScalar    - if the motor powers need to be scaled down to match other motors
     */
    public Motor(String name, boolean reversed, boolean encoder, double tickMultiplier, boolean brake, double maximumCurrent, double powerScalar) {

        boolean done = false;
        for (Motor motorCheck: motors) {
            if (motorCheck.motorName.equals(name)) {
                done = true;
                this.power =  motorCheck.power;

            }
        }

        motors.add(this);
        motorName = name;
        motor = (DcMotorImplEx) BaseOpMode.hardware.dcMotor.get(name);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        if (brake) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (reversed) this.powerScalar = -Math.abs(powerScalar);
        else this.powerScalar = Math.abs(powerScalar);

        if (encoder) this.encoder = new MotorEncoder(name, tickMultiplier);

        motor.setCurrentAlert(maximumCurrent, CurrentUnit.MILLIAMPS);
    }

    public Motor(String name, boolean reversed, boolean encoder, double tickMultiplier, boolean brake) {
        this(name, reversed, encoder, tickMultiplier, brake, 10000, 1);
    }

    public Motor(String name, boolean reversed, boolean encoder, double tickMultiplier) {
        this(name, reversed, encoder, tickMultiplier, true, 10000, 1);
    }

    public Motor(String name, boolean reversed, boolean encoder) {
        this(name, reversed, encoder, 1, true, 10000, 1);
    }

    public Motor(String name, boolean reversed) {
        this(name, reversed, false, 1, true, 10000, 1);
    }

    public Motor(String name) {
        this(name, false, false, 1, true, 10000, 1);
    }

    public void pair(Motor motor) {
        pairedMotors.add(motor);
    }

    public void unpair(Motor motor) {
        pairedMotors.remove(motor);
    }

    private void checkMaximumLoad() {
        if (motor.isOverCurrent()) {
            motor.setMotorDisable();
            currentOverloadTimer.reset();
        } else if (currentOverloadTimer.milliseconds() > 300) {
            motor.setMotorEnable();
        }
    }

    private void commandPower() {
        double setPower = power * powerScalar;

        motor.setPower(setPower);
    }

    public static void printMotorList() {
        System.out.println("Motors:");
        for (Motor m : motors) {
            System.out.println(m.motorName);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        motor.setZeroPowerBehavior(mode);
    }


    public double getVelocity(){
        return motor.getVelocity();
    }

    public double getPower(){return power;}

}
