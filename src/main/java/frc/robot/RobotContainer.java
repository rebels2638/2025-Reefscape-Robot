package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.AutoRunner;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.elevator.RunElevatorRaw;
import frc.robot.commands.autoAlignment.complex.*;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.commands.roller.*;

public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }

        return instance;
    }

    private final SwerveDrive swerveDrive;
    private final Vision vision;
    private final RobotState robotState;

    private final Pivot pivot;
    // private final Claw claw;
    // private final Claw claw;

    private final Elevator elevator;
    private final Roller roller;
    private final MechanismVisualizer mechanismVisualizer;

    private final AutoRunner autoRunner;

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        swerveDrive = SwerveDrive.getInstance();
        vision = Vision.getInstance();
        robotState = RobotState.getInstance();
        pivot = Pivot.getInstance();
        // claw = Claw.getInstance();
        // claw = Claw.getInstance();

        roller = Roller.getInstance();
        elevator = Elevator.getInstance();

        mechanismVisualizer = MechanismVisualizer.getInstance();
        
        NamedCommands.registerCommand("Intake", new IntakeCoral());

        autoRunner = AutoRunner.getInstance();

        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
        // pivot.setDefaultCommand(new RunPivotRaw(xboxOperator));
        // pivot.setDefaultCommand(new RunPivotRaw(xboxOperator));
        xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));

        xboxDriver.getLeftBumper().whileTrue(new AlignToLeftBranch());
        xboxDriver.getRightBumper().whileTrue(new AlignToRightBranch());
        // xboxDriver.getYButton().whileTrue(new AlignToAlgay());
        // xboxDriver.getAButton().whileTrue(new RunClawEject());

        xboxDriver.getAButton().onTrue(new MovePivotAlgay());
        xboxDriver.getBButton().onTrue(new MovePivotStow());

        // xboxDriver.getYButton().onTrue(new IntakeCoral());

        xboxOperator.getLeftBumper().onTrue(new IntakeCoral());
        xboxOperator.getRightBumper().onTrue(new EjectCoral());

        // xboxOperator.getLeftBumper().onTrue(new RunClawIntake());
        // xboxOperator.getRightBumper().onTrue(new StopClaw());

        // xboxOperator.getLeftBumper().onTrue(new RunClawIntake());
        // xboxOperator.getRightBumper().onTrue(new StopClaw());

        // elevator.setDefaultCommand(new RunElevatorRaw(xboxOperator));
        xboxOperator.getAButton().onTrue(new MoveElevatorStow());
        xboxOperator.getBButton().onTrue(new MoveElevatorL2());
        xboxOperator.getYButton().onTrue(new MoveElevatorL3());
        xboxOperator.getXButton().onTrue(new MoveElevatorL4());

        // pivot.setDefaultCommand(new RunPivotRaw(xboxOperator));

        // xboxDriver.getLeftBumper().whileTrue(new LinearDriveToPose(() ->
        // robotState.getClosestLeftBranchPose(), () -> new ChassisSpeeds()));
        // xboxDriver.getRightBumper().whileTrue(new LinearDriveToPose(() ->
        // robotState.getClosestRightBranchPose(), () -> new ChassisSpeeds()));
        // xboxDriver.getYButton().whileTrue(new LinearDriveToPose(() ->
        // robotState.getClosestAlgayPose(), () -> new ChassisSpeeds()));

        // xboxDriver.getYButton().onTrue(new
        // PathplanToPose(RobotState.getInstance().alignmentPoseSearch()));
    }

    public Command getAutonomousCommand() {
        return autoRunner.getAutonomousCommand();
    }
}
