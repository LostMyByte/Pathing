package org.firstinspires.ftc.teamcode.teamcode.Motion.Paths;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ReferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class PathBuilder extends ReferenceSignal {

    SystemModel model;
    java.util.Vector<Path> segments;
    java.util.Vector<Double> times;

    public PathBuilder(SystemModel model) {
        super(model.getDimensions());
        this.model = model;
    }

    @Override
    public Vector predict(double time) {
        double current = 0;
        for (int i = 0; i < segments.size(); i++) {
            current = current + times.get(i);
            if (current > time) return model.toStateSpace(segments.get(i).getPosition(time), segments.get(i).getVelocity(time));
        }
        return model.toStateSpace(segments.lastElement().getTarget(), segments.lastElement().getTargetVelocity());
    }

    @Override
    public Vector target() {
        return model.toStateSpace(segments.lastElement().getTarget(), segments.lastElement().getTargetVelocity());
    }

    @Override
    protected void update() {
        this.data = predict(timer.time());
    }

    public void addPath(Path path, double time) {
        this.segments.add(path);
        this.times.add(time);
    }
}
