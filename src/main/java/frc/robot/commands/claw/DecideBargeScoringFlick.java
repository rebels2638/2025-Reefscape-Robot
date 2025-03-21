// package frc.robot.commands.claw;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.commands.claw.simple.RunClawEject;
// import frc.robot.commands.elevator.simple.DequeueElevatorAction;
// import frc.robot.commands.elevator.simple.QueueStowAction;
// import frc.robot.commands.pivot.simple.MovePivotAlgay;
// import frc.robot.commands.pivot.simple.MovePivotMidwayAlgay;
// import frc.robot.commands.pivot.simple.MovePivotStow;
// import frc.robot.commands.pivot.simple.MovePivotBarge;
// import frc.robot.subsystems.claw.Claw;
// import frc.robot.RobotState;

// public class DecideBargeScoringFlick extends ConditionalCommand {
//     public DecideBargeScoringFlick() {
//         super(
//             new SequentialCommandGroup(
//                 new MovePivotStow(),
//                 new DequeueElevatorAction(),
//                 new MovePivotAlgay(),
//                 new RunClawEject(),
//                 new QueueStowAction(),
//                 new MovePivotBarge(),
//                 new DequeueElevatorAction()
//             ), 
//             new SequentialCommandGroup(
//                 new MovePivotMidwayAlgay(),
//                 new DequeueElevatorAction(),
//                 new MovePivotBarge(),
//                 new RunClawEject(),
//                 new QueueStowAction(),
//                 new DequeueElevatorAction()
//             ), 
//             () -> MathUtil.isNear(0, RobotState.getInstance().getEstimatedPose().getRotation().getDegrees(), 90) //&& Claw.getInstance().inClaw()
//         ); 
//     }
// }
