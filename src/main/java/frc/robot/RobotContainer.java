package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.DoSomething;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.autoAlignment.*;
import frc.robot.constants.Constants.AlignmentConstants;
public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }

        return instance;
    }

    private final SwerveDrive swerveDrive;
    private final AutoRunner autoRunner;
    private final Vision vision;
    private final RobotState robotState;
    // private final Elevator elevator;

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
        autoRunner = AutoRunner.getInstance();

        // elevator = Elevator.getInstance();

        // elevator.setDefaultCommand(new RunElevatorRaw(xboxDriver));

        // auto commands before others
        NamedCommands.registerCommand("DoSomething", new DoSomething());


        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));

        // 3,4,0

        xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));

        xboxDriver.getLeftBumper().whileTrue( 
            new ConditionalCommand(
                new LinearDriveToPose(() -> robotState.getClosestLeftBranchPose(), () -> new ChassisSpeeds()),
                new InstantCommand(),
                () -> robotState.getClosestLeftBranchPose().getTranslation().getDistance(robotState.getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS
            )
        );

        xboxDriver.getRightBumper().whileTrue(
            new ConditionalCommand(
                new LinearDriveToPose(() -> robotState.getClosestLeftBranchPose(), () -> new ChassisSpeeds()),
                new InstantCommand(),
                () -> robotState.getClosestRightBranchPose().getTranslation().getDistance(robotState.getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS
            )
        );

        xboxDriver.getYButton().whileTrue(
            new ConditionalCommand(
                new LinearDriveToPose(() -> robotState.getClosestLeftBranchPose(), () -> new ChassisSpeeds()),
                new InstantCommand(),
                () -> robotState.getClosestAlgayPose().getTranslation().getDistance(robotState.getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS
            )
        );

        // xboxDriver.getYButton().onTrue(new PathplanToPose(RobotState.getInstance().alignmentPoseSearch()));
  }
  
  public Command getAutonomousCommand() {
    return autoRunner.getAutonomousCommand();
  }

}
