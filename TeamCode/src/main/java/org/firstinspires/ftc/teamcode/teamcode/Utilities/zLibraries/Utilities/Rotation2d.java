// Primary Author: Past Team Members
package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities;

public class Rotation2d {
double m_value;
double m_cos;
double m_sin;
    public Rotation2d(double value) {
        while (value > Math.PI) value -= 2 * Math.PI;
        while (value < -Math.PI) value += 2 * Math.PI;
        m_value = value;
        m_cos = Math.cos(value);
        m_sin = Math.sin(value);
    }

    public double getRadians(){
        return m_value;
    }
}
