package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.autoAlignment.*;
public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }

        return instance;
    }

    private final SwerveDrive swerveDrive;
    // private final AutoRunner autoRunner;
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
      // autoRunner = AutoRunner.getInstance();

      // elevator = Elevator.getInstance();

      // elevator.setDefaultCommand(new RunElevatorRaw(xboxDriver));

      swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));

      // 3,4,0

      xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));

      xboxDriver.getLeftBumper().whileTrue(new LinearDriveToPose(() -> robotState.getClosestLeftBranchPose(), () -> new ChassisSpeeds()));
      xboxDriver.getRightBumper().whileTrue(new LinearDriveToPose(() -> robotState.getClosestRightBranchPose(), () -> new ChassisSpeeds()));
      xboxDriver.getYButton().whileTrue(new LinearDriveToPose(() -> robotState.getClosestAlgayPose(), () -> new ChassisSpeeds()));

      // xboxDriver.getYButton().onTrue(new PathplanToPose(RobotState.getInstance().alignmentPoseSearch()));
  }
  
  public Command getAutonomousCommand() {
    // return autoRunner.getAutonomousCommand();
    return null;
  }

}
