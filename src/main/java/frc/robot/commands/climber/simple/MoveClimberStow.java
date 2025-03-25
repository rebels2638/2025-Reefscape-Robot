package frc.robot.commands.climber.simple;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class MoveClimberStow extends Command {
    private final Climber climber;
    public MoveClimberStow() {
        this.climber = Climber.getInstance();
        
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setAngle(Rotation2d.fromDegrees(181)); // 138
    }

    @Override
    public boolean isFinished() {
        return climber.reachedSetpoint();
    }


    @Override
    public void end(boolean interrupted) {

    }
}
