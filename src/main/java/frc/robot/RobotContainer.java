package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.autoAlignment.LinearDriveToPose;
import frc.robot.commands.autoAlignment.PathplanToPose;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.constants.MechAElementConstants;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import com.pathplanner.lib.util.FlippingUtil;
  
public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  
  private final SwerveDrive swerveDrive;
  private final Vision vision;
  // private final Roller roller;

  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    swerveDrive = SwerveDrive.getInstance();
    vision = Vision.getInstance();
    // roller = Roller.getInstance();

    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));

    // xboxOperator.getAButton().onTrue(new IntakeCoral());
    // xboxOperator.getBButton().onTrue(new EjectCoral());
    
    xboxDriver.getAButton().whileTrue(new LinearDriveToPose(new Pose2d(5, 5, new Rotation2d(3)), new ChassisSpeeds(0, 0, 0)));
    xboxDriver.getXButton().onTrue(new InstantCommand(() -> RobotState.getInstance().zeroGyro()));
    xboxDriver.getYButton().onTrue(new PathplanToPose(RobotState.getInstance().alignmentPoseSearch()));
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }
  
  public Command getAutonomousCommand() {
    return (Command) null;
  }

}

