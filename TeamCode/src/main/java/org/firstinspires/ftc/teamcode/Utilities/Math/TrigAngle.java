package org.firstinspires.ftc.teamcode.Utilities.Math;

public class TrigAngle {

    public double cos;
    public double sin;
    public double angle;

    private static double recent = 0;
    private static double recentcos = 1;
    private static double recentsin = 0;

    /**
     * Constructs a new TrigAngle
     * @param angle Angle Measure (Radians)
     */
    public TrigAngle(double angle) {
        this.angle = angle;
        this.cos = Math.cos(angle);
        this.sin = Math.sin(angle);
        /*
        if (angle == recent) {
            this.angle = angle;
            this.cos = recentcos;
            this.sin = recentsin;
        }

        else {

            recentcos = this.cos;
            recentsin = this.sin;
            recent = angle;
        }*/

    }
}
