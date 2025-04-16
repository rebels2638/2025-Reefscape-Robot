package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RebelUtil {
    public static final double EPSILON = 1e-12;
    /**
     * Constrains a number to be within a minimum and maximum
     * 
     * @param toConstrain The number you're constraining
     * @param min The minimum value allowable for this number
     * @param max The maximum value allowable for this number
     * 
     * @return the constrained value (will be between min and max)
     */
    public static double constrain(double toConstrain, double min, double max) {
        if (toConstrain > max) {
            return max;
        }
        if (toConstrain < min) {
            return min;
        }
        return toConstrain;
    }

    public static double linearDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } 
            else {
                return (value + deadband) / (1.0 - deadband);
            }
        } 
        else {
            return 0.0;
        }
    }

    public static double cubicDeadband(double value, double deadband, double weight) {
        value = linearDeadband(value, deadband);
        return weight * Math.pow(value, 3) + (1 - weight) * value;
    }

    /**
     * Checks if a number is "close enough" for equality to another number.
     * 
     * @param a The first number you want to check
     * @param b The second number you want to check
     * @param epsilon Allowed difference
     * 
     * @return true/false for whether equality is met
     * 
     * Second version uses default EPSILON
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static ChassisSpeeds scaleSpeeds(double maxTranslationalSpeedMps, ChassisSpeeds speeds) {
        ChassisSpeeds corrected = new ChassisSpeeds();
        double scaler = Math.sqrt(Math.pow(maxTranslationalSpeedMps,2)/(Math.pow(speeds.vxMetersPerSecond,2) + Math.pow(speeds.vyMetersPerSecond,2)));
        corrected.vxMetersPerSecond = speeds.vxMetersPerSecond * scaler;
        corrected.vyMetersPerSecond = speeds.vyMetersPerSecond * scaler;
        corrected.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
        return corrected;
    }

    public static Rotation2d subtractRotations(Rotation2d a, Rotation2d b) {
        return new Rotation2d(Math.min(Math.abs(a.getRadians() - b.getRadians()), Math.PI * 2 - Math.abs(a.getRadians() - b.getRadians())));
    }
}