package frc.robot.commands.pivot.simple;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.pivot.PivotConfigComp;
import frc.robot.subsystems.pivot.Pivot;

public class MovePivotBargeForwards extends Command {
    private final Pivot pivot;
    public MovePivotBargeForwards() {
        this.pivot = Pivot.getInstance();
        
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setAngle(Rotation2d.fromDegrees(70));//110
    }

    @Override
    public boolean isFinished() {
        return pivot.reachedSetpoint();
    }


    @Override
    public void end(boolean interrupted) {

    }
}
