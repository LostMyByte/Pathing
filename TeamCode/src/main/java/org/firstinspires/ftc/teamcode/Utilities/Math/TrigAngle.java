package org.firstinspires.ftc.teamcode.Utilities.Math;

public class TrigAngle {

    public double cos;
    public double sin;
    public double angle;

    private static TrigAngle recent = new TrigAngle(0);

    /**
     * Constructs a new TrigAngle
     * @param angle Angle Measure (Radians)
     */
    public TrigAngle(double angle) {
        if (angle == recent.angle) {
            this.angle = angle;
            this.cos = recent.cos;
            this.sin = recent.sin;
        }

        else {
            this.angle = angle;
            this.cos = Math.cos(angle);
            this.sin = Math.sin(angle);
            recent = this;
        }

    }
}
