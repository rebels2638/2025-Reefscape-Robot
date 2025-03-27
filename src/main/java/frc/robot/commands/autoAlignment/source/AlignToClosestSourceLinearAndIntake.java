package frc.robot.commands.autoAlignment.source;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.RumbleDriver;
import frc.robot.commands.autoAlignment.LinearAlignSource;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.roller.Roller;

public class AlignToClosestSourceLinearAndIntake extends SequentialCommandGroup {
    public AlignToClosestSourceLinearAndIntake(XboxController controller) {
        addCommands(
            new InstantCommand(() -> RobotState.getInstance().requestGlobalVisionEstimateScale()),
            new ParallelDeadlineGroup(
                new WaitUntilCommand( // we wait for this ot be true to allow continual scheduling
                    () -> AlignmentUtil.getClosestSourcePose().getTranslation().getDistance( // check for the correct max distance from target
                    RobotState.getInstance().getEstimatedPose().getTranslation()) <= 2.3 &&
                    !Roller.getInstance().inRoller()
                ),
                new AbsoluteFieldDrive(controller),
                new RumbleDriver(controller)
            ),
            new ParallelCommandGroup(
                new LinearAlignSource(
                    () -> AlignmentUtil.getClosestSourcePose(),
                    () -> new ChassisSpeeds(),
                    2.3
                ),
                new IntakeCoral()
            )
        );
    }
}