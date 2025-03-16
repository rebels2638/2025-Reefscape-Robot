package frc.robot.commands.autoAlignment.barge;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.climber.simple.MoveClimberStow;
import frc.robot.commands.climber.simple.MoveDeepCage;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;

public class AlignToCageAndClimb extends SequentialCommandGroup{
    public AlignToCageAndClimb(XboxController controller) {
        addCommands(
            new QueueL2Action(),
            new ParallelDeadlineGroup(
                new WaitUntilCommand( // we wait for this ot be true to allow continual scheduling
                    () -> AlignmentUtil.getClosestCagePose().getTranslation().getDistance( // check for the correct max distance from target
                    RobotState.getInstance().getEstimatedPose().getTranslation()) <= 5
                ),
                new AbsoluteFieldDrive(controller)
            ),
            new DequeueElevatorAction(),
            new ParallelCommandGroup(
                new LinearAlign(
                    () -> AlignmentUtil.getClosestCagePoseRecessed(),
                    () -> new ChassisSpeeds(),
                    5
                ),
                new MoveDeepCage()
            ),
            new LinearAlign(
                    () -> AlignmentUtil.getClosestCagePose(),
                    () -> new ChassisSpeeds(),
                    2
            ),
            new MoveClimberStow()
        );
    }
}