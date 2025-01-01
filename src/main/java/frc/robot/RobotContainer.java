package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  public static RobotContainer instance = null;

  public RobotContainer() {

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

