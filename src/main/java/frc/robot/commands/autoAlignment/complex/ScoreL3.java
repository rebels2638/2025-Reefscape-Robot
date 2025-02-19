package frc.robot.commands.autoAlignment.complex;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearDriveToPose;
import frc.robot.commands.autoAlignment.source.AlignToClosestSource;
import frc.robot.commands.elevator.simple.MoveElevatorL3;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.constants.Constants;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.roller.Roller;

public class ScoreL3 extends ConditionalCommand { // Ideal structure
    public ScoreL3() {
        super (
            new ParallelCommandGroup(
                new AlignToClosestSource(),
                new IntakeCoral()
            ).andThen(createTrackToAndDeposit()),
            createTrackToAndDeposit(),
            () -> !Roller.getInstance().inRoller()
        );
    }

    private static SequentialCommandGroup createTrackToAndDeposit() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new isElevatorExtendable(),
                    new MoveElevatorL3()
                ),
                new LinearDriveToPose(
                    () -> AlignmentUtil.decideScoringTarget(3, Constants.GamePiece.CORAL),
                    () -> new ChassisSpeeds()
                )
            ),
            new MoveElevatorL3(),
            new EjectCoral(),
            new MoveElevatorStow()
        );
    }
}