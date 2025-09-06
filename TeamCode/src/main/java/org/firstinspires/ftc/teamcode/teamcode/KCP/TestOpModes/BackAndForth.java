package org.firstinspires.ftc.teamcode.teamcode.KCP.TestOpModes;


import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Forth;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.StopState;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;

@Autonomous(name = "Back and Forth")
@Disabled
public class BackAndForth extends BaseOpMode {

    Movement drive;
    AAA_Paths.Path state = Forth;
    ElapsedTime stateTime;
    double currentV;

    @Override
    public void externalInit() {
        drive = new Movement(0,0,0);
        stateTime = new ElapsedTime();



        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("H", TwoWheelOdometry.heading());
    }

    @Override
    public void externalLoop() {

        stateMachine();
        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("H", TwoWheelOdometry.heading());
    }

    public void stateMachine(){
        switch (state){
            case Forth:
                forth();
                break;
            case StopState:
                stopMoving();
                break;
        }
    }


    public void forth(){
        BaseOpMode.addData("%Done", Forth.t);
        drive.holdPosition(0,0,0, .55);
        if(stateTime.seconds() > 10){
            setState(StopState);
        }
    }

    public void stopMoving(){
        drive.holdPosition(0,50,0, .55);
        if(stateTime.seconds() > 10){
            setState(Forth);
        }

        BaseOpMode.addData("%Done", StopState.t);
    }

    public void setState(AAA_Paths.Path state){
        this.state = state;
        stateTime.reset();
    }
}
