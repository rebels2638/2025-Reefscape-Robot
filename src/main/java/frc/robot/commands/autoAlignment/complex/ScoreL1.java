package frc.robot.commands.autoAlignment.complex;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearDriveToPose;
import frc.robot.commands.autoAlignment.source.AlignToClosestSourceLinear;
import frc.robot.commands.elevator.simple.MoveElevatorL1;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.roller.Roller;

public class ScoreL1 extends ConditionalCommand { // Ideal structure
    public ScoreL1() {
        super (
            new ParallelCommandGroup(
                new AlignToClosestSourceLinear(),
                new IntakeCoral()
            ).andThen(trackToAndDeposit),
            trackToAndDeposit,
            () -> !Roller.getInstance().inRoller()
        );
    }

    private static final SequentialCommandGroup trackToAndDeposit = new SequentialCommandGroup (
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new isElevatorExtendable(),
                new MoveElevatorL1()
            ),
            new LinearDriveToPose(
                () -> AlignmentUtil.decideScoringTarget(),
                () -> new ChassisSpeeds())
        ),
        new MoveElevatorL1(),
        new EjectCoral(),
        new MoveElevatorStow()
    );
}