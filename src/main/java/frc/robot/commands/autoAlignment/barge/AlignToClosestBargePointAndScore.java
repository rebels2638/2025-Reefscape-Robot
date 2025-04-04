// package frc.robot.commands.autoAlignment.barge;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.RobotState;
// import frc.robot.commands.AbsoluteFieldDrive;
// import frc.robot.commands.autoAlignment.LinearAlignFace;
// import frc.robot.commands.autoAlignment.LinearDriveToPose;
// import frc.robot.commands.claw.simple.RunClawEject;
// import frc.robot.commands.elevator.simple.DequeueElevatorAction;
// import frc.robot.commands.elevator.simple.QueueL4Action;
// import frc.robot.commands.elevator.simple.QueueStowAction;
// import frc.robot.commands.pivot.simple.MovePivotAlgay;
// import frc.robot.commands.pivot.simple.MovePivotBargeBackwards;
// import frc.robot.commands.pivot.simple.MovePivotBargeForwards;
// import frc.robot.commands.pivot.simple.MovePivotMidwayAlgay;
// import frc.robot.commands.pivot.simple.MovePivotStow;
// import frc.robot.lib.input.XboxController;
// import frc.robot.lib.util.AlignmentUtil;
// import frc.robot.subsystems.claw.Claw;
// import frc.robot.subsystems.pivot.Pivot;

// public class AlignToClosestBargePointAndScore extends SequentialCommandGroup{
//     public AlignToClosestBargePointAndScore(XboxController controller) {
//         addCommands(
//             new InstantCommand( () -> RobotState.getInstance().requestGlobalVisionEstimateScale() ),
//             new ParallelDeadlineGroup(
//                 new WaitUntilCommand(
//                     () -> AlignmentUtil.getClosestBargePose().getTranslation().getDistance(
//                         RobotState.getInstance().getEstimatedPose().getTranslation()) <= 1 &&
//                         Claw.getInstance().inClaw()
//                 ),
//                 new AbsoluteFieldDrive(controller)
//             ),
//             new LinearDriveToPose(() -> AlignmentUtil.getClosestBargePose(), () -> new ChassisSpeeds()), // drive to a intermediate pose   
//             // new MoveSuperstructureBargeSequence()
//         );
//         Logger.recordOutput("AlignToBargeAxisLocked", AlignmentUtil.getClosestBargePose());
//     }
// }