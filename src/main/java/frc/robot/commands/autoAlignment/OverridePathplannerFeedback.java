package frc.robot.commands.autoAlignment;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class OverridePathplannerFeedback extends Command {
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private static ChassisSpeeds mostRecentSpeeds = new ChassisSpeeds();
    /*
     * @param speedsSupplier A supplier that supplies the speeds to override the pathplanner feedback with - field relative
     */
    public OverridePathplannerFeedback(Supplier<ChassisSpeeds> speedsSupplier) {
        this.speedsSupplier = speedsSupplier;
    }

    @Override
    public void execute() {
        Logger.recordOutput("OverridePathplannerFeedback/speedsSupplier", speedsSupplier.get());

        if (speedsSupplier.get().vxMetersPerSecond == 0 && 
            speedsSupplier.get().vyMetersPerSecond == 0 && 
            speedsSupplier.get().omegaRadiansPerSecond == 0) {

            // Clear all feedback overrides
            PPHolonomicDriveController.clearFeedbackOverrides();

            Logger.recordOutput("OverridePathplannerFeedback/clearing", true);
        }
        else {
            // Override the X feedback
            PPHolonomicDriveController.overrideXFeedback(() -> {
                return speedsSupplier.get().vxMetersPerSecond;
            });

            // Override the Y feedback
            PPHolonomicDriveController.overrideYFeedback(() -> {
                // Calculate feedback from your custom PID controller
                return speedsSupplier.get().vyMetersPerSecond;
            });

            // Override the rotation feedback
            PPHolonomicDriveController.overrideRotationFeedback(() -> {
                // Calculate feedback from your custom PID controller
                return speedsSupplier.get().omegaRadiansPerSecond;
            });

            Logger.recordOutput("OverridePathplannerFeedback/clearing", false);
        }

        mostRecentSpeeds = speedsSupplier.get();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Clear all feedback overrides
        PPHolonomicDriveController.clearFeedbackOverrides();

        mostRecentSpeeds = new ChassisSpeeds();
    }

    public static double getTranslationOverrideMagnitude() {
        return Math.hypot(
            mostRecentSpeeds.vxMetersPerSecond,
            mostRecentSpeeds.vyMetersPerSecond
        );
    }
}