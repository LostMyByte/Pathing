package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities;

public abstract class LoopingThread extends Thread{
    private static boolean doLoop = true;

    protected abstract void init();
    protected abstract void loop();
    protected abstract void end();

    public void endLoop(){
        doLoop = false;
    }

    @Override
    public void run() {
        init();
        while(doLoop){
            loop();
        }
        end();
    }
}
