package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;

public class AutonomousCommand extends SequentialCommandGroup {
  /**
   * Creates a sequential autonomous command.
   *
   * @param swerveDrive The swerve drivetrain subsystem.
   */
  public AutonomousCommand(Drivetrain swerveDrive) {
    // Drive forward at 1.0 m/s for 2 seconds.
    addCommands(
        new RunCommand(() -> swerveDrive.drive(1.0, 0.0, 0.0), swerveDrive).withTimeout(2.0),
        // Turn in place (rotate at 0.5 rad/s) for 1 second.
        new RunCommand(() -> swerveDrive.drive(0.0, 0.0, 0.5), swerveDrive).withTimeout(1.0),
        // Drive backward at -1.0 m/s for 2 seconds.
        new RunCommand(() -> swerveDrive.drive(-1.0, 0.0, 0.0), swerveDrive).withTimeout(2.0),
        // Stop the robot.
        new RunCommand(() -> swerveDrive.drive(0.0, 0.0, 0.0), swerveDrive)
    );
  }
}
