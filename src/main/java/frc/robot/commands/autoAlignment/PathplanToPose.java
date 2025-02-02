package frc.robot.commands.autoAlignment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.constants.MechAElementConstants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;

public class PathPlanToPose extends Command {
    private Command followPathHolonomic;
    private final Pose2d target;

    public PathPlanToPose(Pose2d target) {
        this.target = target;
    }

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

    public static Pose2d alignmentPoseSearch() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d current = RobotState.getInstance().getEstimatedPose();
        List<Pose2d> candidates = new ArrayList<>(
            Arrays.asList(
                MechAElementConstants.Processor.centerFace,
                new Pose2d(MechAElementConstants.Barge.farCage, new Rotation2d(0)),
                new Pose2d(MechAElementConstants.Barge.middleCage, new Rotation2d(0)),
                new Pose2d(MechAElementConstants.Barge.closeCage, new Rotation2d(0)),
                MechAElementConstants.CoralStation.leftCenterFace,
                MechAElementConstants.CoralStation.rightCenterFace,
                MechAElementConstants.StagingPositions.leftIceCream,
                MechAElementConstants.StagingPositions.middleIceCream,
                MechAElementConstants.StagingPositions.rightIceCream
            )
          );
    
        for (Pose2d element : MechAElementConstants.Reef.centerFaces) {candidates.add(element);}
    
        return alliance.isPresent() ? 
          alliance.get() == DriverStation.Alliance.Blue ?
            current.nearest(candidates) : current
              .nearest(
                candidates.stream()
                .map(
                  FlippingUtil::flipFieldPose)
                    .collect(Collectors.toList())
              )
         : current;
      }
}