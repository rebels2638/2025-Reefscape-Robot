package frc.robot.commands.autoAlignment;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlignmentConstants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;

public class AlignmentUtil {
    private static final SwerveDrivetrainConfigBase config;
    static {
        switch (Constants.currentMode) {
            case COMP:
                config = SwerveDrivetrainConfigComp.getInstance();

                break;

            case PROTO:
                config = SwerveDrivetrainConfigProto.getInstance();

                break;
            
            case SIM:
                config = SwerveDrivetrainConfigSim.getInstance();

                break;

            case REPLAY:
                config = SwerveDrivetrainConfigComp.getInstance();

                break;

            default:
                config = SwerveDrivetrainConfigComp.getInstance();

                break;
        }
    }
    
    public static Pose2d offsetPoseToPreAlignment(Pose2d pose) {
        return pose.transformBy(
            new Transform2d(
                -config.getBumperLengthMeters() / 2 * Math.sqrt(2), // this is the bumper radius
                0,
                new Rotation2d(0)
            )
        );
    }

    public static Pose2d offsetBranchPose(Pose2d pose, boolean isLeftBranch) {
      double bumperOffset = config.getBumperLengthMeters() / 2;
      double invert = isLeftBranch ? 1 : -1;

      return pose.transformBy(
        new Transform2d(
          -bumperOffset + config.getBranchOffsetFromRobotCenter().getX(),
          invert * (AlignmentConstants.kINTER_BRANCH_DIST_METER / 2) + config.getBranchOffsetFromRobotCenter().getY(),
          new Rotation2d(0)
        )
      );
  }

  public static int getClosestFace(Pose2d curr, List<Pose2d> candidates) {
    int nearest = 0;
    int penultimateNearest = 0;
    for (int i = 0; i < AlignmentConstants.kCENTER_FACES.length; i++) {
      if (AlignmentConstants.kCENTER_FACES[i].getTranslation().getDistance(curr.getTranslation()) < 
          AlignmentConstants.kCENTER_FACES[nearest].getTranslation().getDistance(curr.getTranslation())
      ) {
        nearest = i;
      }
    }

    for (int i = 0; i < AlignmentConstants.kCENTER_FACES.length; i++) {
      if (i == nearest) {continue;}
      if (AlignmentConstants.kCENTER_FACES[i].getTranslation().getDistance(curr.getTranslation()) < 
          AlignmentConstants.kCENTER_FACES[nearest].getTranslation().getDistance(curr.getTranslation())
      ) {
        penultimateNearest = i;
      }
    }

    return candidates.get(nearest).getTranslation().getDistance(curr.getTranslation()) < 
      candidates.get(penultimateNearest).getTranslation().getDistance(curr.getTranslation()) ?
        nearest : penultimateNearest;
  }

  public static Pose2d getClosestAlgayPose() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Pose2d current = RobotState.getInstance().getEstimatedPose();
    List<Pose2d> candidates = new ArrayList<>();

    double bumperOffset = config.getBumperLengthMeters() / 2;

    for (Pose2d element : AlignmentConstants.kCENTER_FACES) {
      candidates.add(
        element.transformBy(
          new Transform2d(
            config.getAlgayOffsetFromRobotCenter().getX() - bumperOffset,
            config.getAlgayOffsetFromRobotCenter().getY(),
            new Rotation2d(0)
          )
        )
      );
    }

    Pose2d nearest = candidates.get(getClosestFace(current, candidates));
    
    nearest = alliance.isPresent() ? 
      alliance.get() == DriverStation.Alliance.Blue ?
        nearest : 
        FlippingUtil.flipFieldPose(nearest)
    : nearest;

    Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
    return nearest;
  }

  public static Pose2d getClosestLeftBranchPose() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Pose2d current = RobotState.getInstance().getEstimatedPose();
    List<Pose2d> candidates = new ArrayList<>();
  
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[0], true));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[1], false));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[2], true));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[3], false));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[4], true));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[5], false));

    Pose2d nearest = candidates.get(getClosestFace(current, candidates));
    
    nearest = alliance.isPresent() ? 
      alliance.get() == DriverStation.Alliance.Blue ?
        nearest : 
        FlippingUtil.flipFieldPose(nearest)
    : nearest;

    Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
    return nearest;
  }

  public static Pose2d getClosestRightBranchPose() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Pose2d current = RobotState.getInstance().getEstimatedPose();
    List<Pose2d> candidates = new ArrayList<>();

    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[0], false));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[1], true));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[2], false));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[3], true));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[4], false));
    candidates.add(offsetBranchPose(AlignmentConstants.kCENTER_FACES[5], true));

    Pose2d nearest = candidates.get(getClosestFace(current, candidates));

    nearest = alliance.isPresent() ? 
      alliance.get() == DriverStation.Alliance.Blue ?
        nearest : 
        FlippingUtil.flipFieldPose(nearest)
    : nearest;

    Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
    return nearest;
  }
}
