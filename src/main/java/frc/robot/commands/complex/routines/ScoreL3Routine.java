package frc.robot.commands.complex.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearDriveToPose;
import frc.robot.commands.autoAlignment.source.AlignToClosestSource;
import frc.robot.commands.elevator.simple.MoveElevatorL3;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.constants.Constants;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;

public class ScoreL3Routine extends ConditionalCommand { // Ideal structure
    private static Pose2d commandTarget;

    public ScoreL3Routine() {
        super(
            new ParallelCommandGroup(
                new AlignToClosestSource(),
                new IntakeCoral()
            ).andThen(createTrackToAndDeposit()),
            createTrackToAndDeposit(),
            () -> !Roller.getInstance().inRoller()
        );

        Roller.getInstance().isScored(RobotState.getInstance().getScoredPosition(commandTarget), Elevator.getInstance().getElevatorHeight());
    }

    private static SequentialCommandGroup createTrackToAndDeposit() {
        commandTarget = AlignmentUtil.decideScoringTargetMaxPoints(3, Constants.GamePiece.CORAL);
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new isElevatorExtendable(),
                    new MoveElevatorL3()
                ),
                new LinearDriveToPose(
                    () -> commandTarget,
                    () -> new ChassisSpeeds()
                )
            ),
            new MoveElevatorL3(),
            new EjectCoral(),
            new MoveElevatorStow()
        );
    }
}