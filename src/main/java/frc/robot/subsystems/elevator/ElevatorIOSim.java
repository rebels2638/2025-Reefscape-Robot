package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIOSim extends SubsystemBase implements ElevatorIO {

    private double shooterHeightMeters;
    private double climberHeightMeters;
    private static final double kSECOND_STAGE_TO_THIRD = 2;

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.shooterHeightMeters = shooterHeightMeters;
        inputs.climberHeightMeters = climberHeightMeters;
    }

    public void setHeightMeters(double goalPositionMeters, boolean isShooterHight, boolean isClimbing) {
        if (isShooterHight) {
            shooterHeightMeters = goalPositionMeters;
            climberHeightMeters = shooterHeightMeters * kSECOND_STAGE_TO_THIRD;
        }
        else {
            climberHeightMeters = goalPositionMeters;
            shooterHeightMeters = climberHeightMeters / kSECOND_STAGE_TO_THIRD;
        }
    }

    public void configureController(ElevatorFeedforward pff, PIDController pfb, ElevatorFeedforward cff, double kCLIMB_KG) {}

    public void setVoltage(double voltage) {}

    public boolean reachedSetpoint() {
        return true;
    }

    public void zeroHeight() {}
}