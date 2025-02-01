package frc.robot.commands.autoAligment;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;

public class AutoAlign extends Command {
    private Command followPathHolonomic;
    Pose2d target;

    public AutoAlign(Pose2d target) {this.target = target;}

    // Builds a follow path holonomic path using field constants.
    @Override
    public void initialize() {
        followPathHolonomic = AutoBuilder.pathfindToPose(
            target, 
            new PathConstraints(
                SwerveDrivetrainConfigComp.getInstance().getMaxDrivetrainTranslationalVelocityMetersPerSec(), 
                SwerveDrivetrainConfigComp.getInstance().getMaxDrivetrainTranslationalAccelerationMetersPerSecSec(), 
                SwerveDrivetrainConfigComp.getInstance().getMaxDrivetrainAngularVelocityRadiansPerSec(), 
                SwerveDrivetrainConfigComp.getInstance().getMaxDrivetrainAngularAccelerationRadiansPerSecSec()
                )
            );

        CommandScheduler.getInstance().schedule(followPathHolonomic);
    }

    @Override   
    public boolean isFinished() {
        // System.out.println(followPathHolonomic.isFinished());
        return followPathHolonomic.isFinished();
    }
}