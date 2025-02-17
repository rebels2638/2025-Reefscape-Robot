package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj.Timer;

import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.Set;
import java.util.TreeMap;

public class TimeDiscreteBooleanBuffer {

    private final NavigableMap<Double, Boolean> buffer = new TreeMap<>();
    private final double retentionTime;

    public TimeDiscreteBooleanBuffer(double retentionTime) {
        this.retentionTime = retentionTime;
    }

    public void addValue(boolean value) {
        double timestamp = Timer.getFPGATimestamp();
        buffer.put(timestamp, value);
        cleanup();
    }

    public boolean getValue(double timestamp) {
        Map.Entry<Double, Boolean> lower = buffer.floorEntry(timestamp);
        Map.Entry<Double, Boolean> higher = buffer.ceilingEntry(timestamp);

        if (lower == null && higher == null) {
            return false;
        }

        if (lower == null) {return higher.getValue();}
        if (higher == null) {return lower.getValue();}

        return (timestamp - lower.getKey()) <= (higher.getKey() - timestamp) ? lower.getValue() : higher.getValue();
    }

    private void cleanup() {
        double cutoff = Timer.getFPGATimestamp() - retentionTime;
        buffer.headMap(cutoff, true).clear();
    }

    public int getSampleCount(double start, double end) {
        return (int) buffer.subMap(start, true, end, true).size();
    }

    public int size() {
        return buffer.size();
    }

    public Set<Entry<Double, Boolean>> getEntrySet() {
        return buffer.entrySet();
    }
}