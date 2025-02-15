// package frc.robot.commands.autoAlignment.source;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.RobotState;
// import frc.robot.commands.autoAlignment.LinearDriveToPose;
// import frc.robot.constants.Constants.AlignmentConstants;
// import frc.robot.lib.util.AlignmentUtil;
// public class AlignToClosestSource extends ConditionalCommand{
//     public AlignToClosestSource() {
//         super(
//             new SequentialCommandGroup ( // align
//                 new ConditionalCommand(
//                     new LinearDriveToPose(() -> AlignmentUtil.offsetPoseToPreAlignment(AlignmentUtil.getClosestValidSource()), () -> new ChassisSpeeds()), // drive to a intermediate pose 
//                     new InstantCommand(), // let the robot drive regularly
//                     () -> Math.abs(RobotState.getInstance().getEstimatedPose().getRotation().minus(AlignmentUtil.getClosestValidSource().getRotation()).getDegrees()) > 45 // check if rotating will first increase the bumper profile while driving to the goal
//                 ),
//                 new LinearDriveToPose(() -> AlignmentUtil.getClosestValidSource(), () -> new ChassisSpeeds()) // finally drive to the target end goal
//             ),
//             new InstantCommand(), // if too far, do nothing
//             () -> AlignmentUtil.getClosestValidSource().getTranslation().getDistance( // check for the correct max distance from target
//                 RobotState.getInstance().getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS
//         );
//     }
// }