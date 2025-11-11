// Primary Author: Caroline Oringer

package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hopper.HopperDash.downTransferPos;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hopper.HopperDash.ki;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hopper.HopperDash.intakePosToShootPos;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hopper.HopperDash.ticksToMoveOneSlot;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hopper.HopperDash.ticksToMoveTwoSlot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;

public class Hopper extends Subsystem {

    CRServo spindexer;
    Servos.Transfer transfer;
    HopperStates hopperState;
    ElapsedTime timer = new ElapsedTime();
    private DcMotor encoder;
    private PID encoderticks;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime onTargetTimer = new ElapsedTime();

    // Targeting
    private double targetTicks = 0;
    private double toleranceTicks = 10;
    private boolean holdWhenReached = false; // if true, hold with small power bias at target


    public Hopper(HardwareMap hardwareMap) {
        hopperState = HopperStates.NOTACTIVE;

        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        transfer  = new Servos.Transfer();

        encoder = hardwareMap.get(DcMotor.class, "encoder");
        encoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoder.setDirection(DcMotorSimple.Direction.FORWARD);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderticks = new PID(HopperDash.kp, HopperDash.ki, HopperDash.kd);
        encoderticks.setFeedForward(HopperDash.kf);
        encoderticks.setLowerLimit(HopperDash.minPower);
        encoderticks.setDeadZone(HopperDash.pidDeadZone);

        toleranceTicks = HopperDash.targetTolerance;

    }

    public void setTargetTicks(double ticks) {
        setTargetTicks(ticks, HopperDash.targetTolerance, false);
    }

    public void setTargetTicks(double ticks, double tolerance, boolean holdAtTarget) {
        this.targetTicks = ticks;
        this.toleranceTicks = Math.max(0, tolerance);
        this.holdWhenReached = holdAtTarget;

        encoderticks.reset();
        onTargetTimer.reset();
        spindexer.setPower(0);
        stateTimer.reset();
        setState(HopperStates.GOTO_TICKS);
    }

    public boolean moveToTicksBlocking(int ticks, long timeoutMs) {
        setTargetTicks(ticks);
        ElapsedTime t = new ElapsedTime();
        while (getState() == HopperStates.GOTO_TICKS && t.milliseconds() < timeoutMs) {
            updateSensors();
            update();
        }
        return getState() != HopperStates.GOTO_TICKS;
    }

    public boolean atTarget() {
        double err = targetTicks - getCurrentTicks();
        return Math.abs(err) <= toleranceTicks;
    }

    public int getCurrentTicks() {
        return encoder.getCurrentPosition();
    }
    @Config
    public static class HopperDash {
        public static double targetPos =0;
        public static double transferDwell= 300;
        public static double transferPos = 0.9;//0.80;
        public static double downTransferPos = 0.47;//1.00;

        // “Move one slot” time, if you still use timed motion
        public static double timeToMoveOneSlotMs = 600;
        //8192 for full turn
        public static double ticksToMoveOneSlot = (double) 2680;
        public static double ticksToMoveTwoSlot =5650 ;
        // PID constants for tick control (tune in Dashboard)
        public static double intakePosToShootPos = 1320;
        public static double kp = 0.0004;
        public static double ki = 0.0;
        public static double kd =-0.0004;
        public static double kf = -0.0;

        public static double maxPower = 1;
        public static double minPower = 0;
        public static double pidDeadZone = 3;
        public static double targetTolerance = 10; // this must must get changed after tuning lol
        public static double onTargetDwellMs = 120; // must remain within tolerance this long
    }

    public enum HopperStates {
        NOTACTIVE,TRANSFER,MOVEBACKONE,MOVEONE,HOPPERSPINNNNNN, //HOPPERSPINNNNNNN is for testing
        GOTO_TICKS, MOVETWO, MOVEBACKTWO, INTAKEPOSTOSHOOTPOS
    }




    public Hopper.HopperStates getState(){
        return hopperState;
    }

    public void setState(Hopper.HopperStates state){
        stateTimer.reset();
        hopperState = state;
    }
    public void work(){
        switch (hopperState){
            case NOTACTIVE:
               safePos();
                break;
            case TRANSFER:
                transfer();
                break;
            case MOVEBACKONE:
                update();
                setTargetTicks(getCurrentTicks()-ticksToMoveOneSlot, toleranceTicks, false);
               // while (stateTimer.milliseconds() < HopperDash.timeToMoveOneSlotMs) moveOneSlot(true);
                setState(HopperStates.GOTO_TICKS);
                break;
            case MOVEONE:

                update();
                setTargetTicks(getCurrentTicks()+ ticksToMoveOneSlot, toleranceTicks, false);
                //  while (stateTimer.milliseconds() < HopperDash.timeToMoveOneSlotMs) moveOneSlot(false);
                setState(HopperStates.GOTO_TICKS);
                break;
            case MOVETWO:
                update();
                setTargetTicks(getCurrentTicks()+ ticksToMoveTwoSlot, toleranceTicks, false);
                setState(HopperStates.GOTO_TICKS);
                break;
            case MOVEBACKTWO:
                update();
                setTargetTicks(getCurrentTicks() -ticksToMoveTwoSlot, toleranceTicks, false);
                setState(HopperStates.GOTO_TICKS);
                break;
            case GOTO_TICKS:
                update();
                break;
            case HOPPERSPINNNNNN:
                spindexer.setPower(1);
                break;
            case INTAKEPOSTOSHOOTPOS:
                update();
                setTargetTicks(getCurrentTicks()+ intakePosToShootPos);
                setState(HopperStates.GOTO_TICKS);
                break;
        }
    }

    public void transfer(){
        spindexer.setPower(0);
        transfer.setPosition(HopperDash.transferPos);
        if (stateTimer.milliseconds()>= HopperDash.transferDwell){
            setState(HopperStates.NOTACTIVE);
        }
        //1 is all the way down, 0.8~ is transfer
    }


    public void safePos(){
        transfer.setPosition(HopperDash.downTransferPos);
        spindexer.setPower(0);
    }

    @Override
    public void updateSensors() {

    }

    @Override
    public void update() {
        encoderticks.setConstants(HopperDash.kp, ki, HopperDash.kd);
        encoderticks.setFeedForward(HopperDash.kf);
        encoderticks.setLowerLimit(HopperDash.minPower);
        encoderticks.setDeadZone(HopperDash.pidDeadZone);

        if (hopperState != HopperStates.GOTO_TICKS) return;

        final int current = getCurrentTicks();
        final double powerCmd = encoderticks.getCorrection(current, targetTicks);
        transfer.setPosition(downTransferPos);
        // Clamp and apply power to CR servo
        double power = Math.max(-HopperDash.maxPower, Math.min(HopperDash.maxPower, powerCmd));
        spindexer.setPower(power);

        // Check if we're within tolerance; require dwell-time before finishing
        if (Math.abs(targetTicks - current) <= toleranceTicks) {
            if (onTargetTimer.milliseconds() >= HopperDash.onTargetDwellMs) {
                if (!holdWhenReached) {
                    spindexer.setPower(0.0);
                } else {
                    // “Hold” with just feedforward/lower-limit in PID; power already computed.
                }
                setState(HopperStates.NOTACTIVE);
            }
        } else {
            onTargetTimer.reset();
        }
        BaseOpMode.addData("targetPos", targetTicks);
        BaseOpMode.addData("currenttick", getCurrentTicks());
        BaseOpMode.updateTelemetry();
    }
}

