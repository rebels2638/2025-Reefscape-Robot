package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.elevator.simple.WaitForNonStowState;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.roller.Roller;

public class AlignToRightBranchLinearAndScore extends SequentialCommandGroup {
    public AlignToRightBranchLinearAndScore(XboxController controller) {
        addCommands(
            new ParallelDeadlineGroup(
                new WaitUntilCommand( // we wait for this ot be true to allow continual scheduling
                    () -> AlignmentUtil.getClosestLeftBranchPose().getTranslation().getDistance( // check for the correct max distance from target
                    RobotState.getInstance().getEstimatedPose().getTranslation()) <= 5 &&
                    Roller.getInstance().inRoller()
                ),
                new AbsoluteFieldDrive(controller)
            ),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new isElevatorExtendable(),
                        new WaitForNonStowState(),
                        new DequeueElevatorAction()
                    ),
                    new LinearAlign(
                        () -> AlignmentUtil.getClosestLeftBranchPose(),
                        () -> new ChassisSpeeds(),
                        5
                    )
                ),
                new EjectCoral(),
                new QueueStowAction(),
                new DequeueElevatorAction()
            )
        );
    }
}