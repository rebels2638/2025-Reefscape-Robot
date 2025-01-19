package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.autoAligment.DriveToPose;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxOperator;
  private final XboxController xboxDriver;
  
  private final SwerveDrive swerveDrive;

  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    swerveDrive = new SwerveDrive();
    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(swerveDrive, xboxDriver));

    xboxDriver.getBButton().onTrue(new DriveToPose(swerveDrive));
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

