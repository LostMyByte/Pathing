package org.firstinspires.ftc.teamcode.teamcode.Utilities.Control;

public class Interpolater {
    // Function to interpolate a value from one range to another
    public static double interpolate(double value, double start1, double end1, double start2, double end2) {
        // Check if the value is within the first range
        if (value < start1) {
            return start2;
        } else if (value > end1) {
            return end2;
        }

        // Map the value from the first range to the second range
        double percentage = (value - start1) / (end1 - start1);
        double interpolatedValue = start2 + percentage * (end2 - start2);

        return interpolatedValue;
    }
}
