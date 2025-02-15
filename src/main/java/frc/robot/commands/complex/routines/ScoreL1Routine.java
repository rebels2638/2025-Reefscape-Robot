// package frc.robot.commands.complex.routines;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.autoAlignment.LinearDriveToPose;
// import frc.robot.commands.autoAlignment.source.AlignToClosestSource;
// import frc.robot.commands.elevator.simple.MoveElevatorL1;
// import frc.robot.commands.elevator.simple.MoveElevatorStow;
// import frc.robot.commands.roller.EjectCoral;
// import frc.robot.commands.roller.IntakeCoral;
// import frc.robot.lib.util.AlignmentUtil;
// import frc.robot.subsystems.roller.Roller;
// public class ScoreL1Routine extends ConditionalCommand {
//     public ScoreL1Routine() {
//         super(
//             new SequentialCommandGroup(
//                 new ParallelCommandGroup(
//                     new AlignToClosestSource(),
//                     new IntakeCoral()
//                 ),
//                 // new SequentialCommandGroup(
//                 //     new isElevatorExtendable(),
//                 //     new ParallelCommandGroup(
//                 //         new MoveElevatorL1(),
//                 //         new LinearDriveToPose(
//                 //             () -> AlignmentUtil.getClosestLeftBranchPose(), 
//                 //             () -> new ChassisSpeeds())
//                 //     )
//                 // ),
//                 new LinearDriveToPose( // this will be a parallel command group once I figure out the weird elevator check
//                     () -> AlignmentUtil.getClosestLeftBranchPose(), 
//                     () -> new ChassisSpeeds()),
//                 new MoveElevatorL1(),
//                 new EjectCoral(),
//                 new MoveElevatorStow()
//             ),
//             new SequentialCommandGroup(
//                 new LinearDriveToPose( // this will be a parallel command group once I figure out the weird elevator check
//                     () -> AlignmentUtil.getClosestLeftBranchPose(), 
//                     () -> new ChassisSpeeds()),
//                 new MoveElevatorL1(),
//                 new EjectCoral(),
//                 new MoveElevatorStow()
//             ),
//             () -> !Roller.getInstance().inRoller()
//         );
//     }
// }