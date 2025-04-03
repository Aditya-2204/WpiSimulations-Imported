package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.auton.AutonomousCommand;

public class RobotContainer {
  // The swerve drive subsystem.
  private final Drivetrain swerveDrive;
  
  // Autonomous command imported from AutonomousCommand.java.
  private final Command autonomousCommand;

  public RobotContainer() {
    // Retrieve the NetworkTable for swerve data.
    NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");
    
    // Instantiate the swerve drive subsystem.
    swerveDrive = new Drivetrain(swerveTable);
    
    // Set a default command for teleop: drive with example values.
    
    // Instantiate the autonomous sequential command.
    autonomousCommand = new AutonomousCommand(swerveDrive);
    
    // Optionally configure additional button bindings.
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Bind joystick buttons to commands if required.
  }

  /**
   * Returns the autonomous command to run.
   */
  public Command getAutonomousCommand() {
    return autonomousCommand;
  }
}
