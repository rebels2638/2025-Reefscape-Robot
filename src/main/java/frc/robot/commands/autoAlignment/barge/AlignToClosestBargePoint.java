package frc.robot.commands.autoAlignment.barge;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.claw.Claw;

public class AlignToClosestBargePoint extends SequentialCommandGroup{
    public AlignToClosestBargePoint(XboxController controller) {
        addCommands(
            new ParallelDeadlineGroup(
                new WaitUntilCommand(
                () -> AlignmentUtil.getClosestBargePose().getTranslation().getDistance(
                    RobotState.getInstance().getEstimatedPose().getTranslation()) <= 5
                    // Claw.getInstance().inClaw()
                ),
                new AbsoluteFieldDrive(controller)
            ),
            new QueueL4Action(),
            new LinearAlign(
                    () -> AlignmentUtil.getClosestBargePose(),
                    () -> new ChassisSpeeds(),
                    5
            )
        );
        Logger.recordOutput("AlignToBargeAxisLocked", AlignmentUtil.getClosestBargePose());
    }
}