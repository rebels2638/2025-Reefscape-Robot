package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.claw.Claw;
  
public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  
  //private final SwerveDrive swerveDrive;

  private final Claw claw;
  
  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    claw = new Claw();

    xboxOperator.getAButton().onTrue(new InstantCommand(() -> claw.setVoltage(12)));
    xboxOperator.getBButton().onTrue(new InstantCommand(() -> claw.setVoltage(0)));

    //swerveDrive = new SwerveDrive();
    //swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(swerveDrive, xboxDriver));

   // xboxDriver.getXButton().onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));
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

