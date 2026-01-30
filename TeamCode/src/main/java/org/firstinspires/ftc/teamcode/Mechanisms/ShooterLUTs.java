package org.firstinspires.ftc.teamcode.Mechanisms;

import static androidx.core.math.MathUtils.clamp;

import com.seattlesolvers.solverslib.util.InterpLUT;

public final class ShooterLUTs {
    // Use a private final class for type-safe data points (Java 8 compatible). Can't use record.
    private static final class LutPoint {
        private final double distanceInches;
        private final double hoodPos;
        private final double flywheelTicks;
        private final double timeOfFlight;

        private LutPoint(double distanceInches, double hoodPos, double flywheelTicks, double timeOfFlight) {
            this.distanceInches = distanceInches;
            this.hoodPos = hoodPos;
            this.flywheelTicks = flywheelTicks;
            this.timeOfFlight = timeOfFlight;
        }
    }

    // This data is much more readable using a class.
    private static final LutPoint[] POINTS = {
            new LutPoint(0, 0, 0, 0),
            new LutPoint(1, 1, 1, 1),
            new LutPoint(2, 2, 2, 2),
    };

    private static final double MIN_DIST = POINTS[0].distanceInches;
    private static final double MAX_DIST = POINTS[POINTS.length - 1].distanceInches;
    private static final InterpLUT HOOD_FROM_DIST = new InterpLUT();
    private static final InterpLUT FLYWHEEL_FROM_DIST = new InterpLUT();
    private static final InterpLUT TOF_FROM_DIST = new InterpLUT();

    static {
        if (POINTS.length < 2) {
            throw new IllegalStateException("Need at least 2 points for interpolation.");
        }

        // Use LutPoint class.
        double prev = POINTS[0].distanceInches;

        for (int i = 1; i < POINTS.length; i++) {
            double x = POINTS[i].distanceInches;
            if (x <= prev) {
                throw new IllegalStateException("POINTS must be strictly increasing by distance. Bad at index " + i);
            }
            prev = x;
        }

        // Populating the LUTs is now self-documenting. Made more readable.
        for (LutPoint p : POINTS) {
            HOOD_FROM_DIST.add(p.distanceInches, p.hoodPos);
            FLYWHEEL_FROM_DIST.add(p.distanceInches, p.flywheelTicks);
            TOF_FROM_DIST.add(p.distanceInches, p.timeOfFlight);
        }
        HOOD_FROM_DIST.createLUT();
        FLYWHEEL_FROM_DIST.createLUT();
        TOF_FROM_DIST.createLUT();
    }

    public static double getHood(double distance) {
        return HOOD_FROM_DIST.get(clampedDistance(distance));
    }

    public static double getFlywheel(double distance) {
        return FLYWHEEL_FROM_DIST.get(clampedDistance(distance));
    }

    public static double getTOF(double distance) {
        return TOF_FROM_DIST.get(clampedDistance(distance));
    }

    public static boolean inRange(double distance) {
        return distance >= MIN_DIST && distance <= MAX_DIST;
    }

    private static double clampedDistance(double distance) {
        return clamp(distance, MIN_DIST, MAX_DIST);
    }

    public static double getMinDist() { return MIN_DIST; }
    public static double getMaxDist() { return MAX_DIST; }

    // Private constructor (correct implementation)
    private ShooterLUTs() {}
}
