package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import java.util.Map;
import java.util.TreeMap;

public class PitchInterpolator {
    private final TreeMap<Double, Double> lookupTable = new TreeMap<>();

    public PitchInterpolator() {
        // Cheie = unghi pitch, Valoare = valoarea ta (0 - 355)
        lookupTable.put(58.2, 340.0);
        lookupTable.put(67.0, 270.0);
    }

    public double getServoValue(double targetPitchAngle) {
        if (targetPitchAngle <= lookupTable.firstKey()) {
            return lookupTable.firstEntry().getValue();
        }
        if (targetPitchAngle >= lookupTable.lastKey()) {
            return lookupTable.lastEntry().getValue();
        }

        Map.Entry<Double, Double> lower = lookupTable.floorEntry(targetPitchAngle);
        Map.Entry<Double, Double> upper = lookupTable.ceilingEntry(targetPitchAngle);

        if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        double ratio = (targetPitchAngle - lower.getKey()) / (upper.getKey() - lower.getKey());
        return lower.getValue() + ratio * (upper.getValue() - lower.getValue());
    }
}
