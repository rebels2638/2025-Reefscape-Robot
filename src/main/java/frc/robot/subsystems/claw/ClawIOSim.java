package frc.robot.subsystems.claw;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.subsystems.pivot.Pivot;

public class ClawIOSim implements ClawIO {
    private double appliedVolts = 0;

    private double clawRunStartTime = 0;

    public ClawIOSim() {}

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.clawVelocityRadPerSec = appliedVolts * 20;
        inputs.clawAppliedVolts = appliedVolts;

        if (Math.abs(inputs.clawAppliedVolts) == 0) {
            clawRunStartTime = Timer.getTimestamp();
        }
        double timeSinceClawRun = Timer.getTimestamp() - clawRunStartTime;
        
        inputs.inClaw = 
            !inputs.inClaw ?
                new Translation2d(4.48, 4.027).getDistance(RobotState.getInstance().getEstimatedPose().getTranslation()) < 3 &&
                Pivot.getInstance().getAngle().getDegrees() < -20 &&
                timeSinceClawRun >= 0.7 && inputs.clawAppliedVolts > 5 : 
                !(timeSinceClawRun >= 0.7 && inputs.clawAppliedVolts < -5);

    }

    @Override
    public void setTorqueCurrentFOC(double current) {
        appliedVolts = current;
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVolts = voltage;
    }
}
