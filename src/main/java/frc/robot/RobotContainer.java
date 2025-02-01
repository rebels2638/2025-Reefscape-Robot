package frc.robot;

import java.util.List;
import java.util.Optional;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.autoAligment.AutoAlign;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
  
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
    
    xboxDriver.getXButton().onTrue(new InstantCommand(() -> RobotState.getInstance().zeroGyro()));
    xboxDriver.getYButton().onTrue(new AutoAlign(alignmentPoseSearch()));
  }

  public Pose2d alignmentPoseSearch() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Pose2d current = RobotState.getInstance().getEstimatedPose();

    return alliance.isPresent() ? 
      alliance.get() == DriverStation.Alliance.Blue ?
        current.nearest(null) : current.nearest(null) // blue default, red default
     : null;
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

