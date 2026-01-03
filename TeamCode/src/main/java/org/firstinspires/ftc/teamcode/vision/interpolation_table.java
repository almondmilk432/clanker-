package org.firstinspires.ftc.teamcode.vision;

public class interpolation_table {

    // calibration points (meters → RPM)
    private static final double[] distances = {0, 1.0, 1.5, 2.0, 2.5};
    private static final double[] rpms      = {1000, 2200, 2500, 2800, 3000};
    private static final double[] hoods     = {0.05, 0.05, 0.10, 0.17, 0.23};

    public static double rpmForDistance(double d) {
        return interpolate(distances, rpms, d);
    }

    public static double hoodForDistance(double d) {
        return interpolate(distances, hoods, d);
    }

    private static double interpolate(double[] x, double[] y, double input) {
        if (input <= x[0]) return y[0];
        if (input >= x[x.length - 1]) return y[y.length - 1];

        for (int i = 0; i < x.length - 1; i++) {
            if (input >= x[i] && input <= x[i + 1]) {
                double ratio = (input - x[i]) / (x[i + 1] - x[i]);
                return y[i] + ratio * (y[i + 1] - y[i]);
            }
        }
        return y[y.length - 1];
    }
}
