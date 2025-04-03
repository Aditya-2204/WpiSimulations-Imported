package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemCommands.SwerveDriveCommands;

public class LeftStartAuto extends SequentialCommandGroup{

    final double shooterSpeed = 1;

    public LeftStartAuto()
    {
        addCommands(
            new SwerveDriveCommands(0.2,0,0).withTimeout(0.89),/*Original:0.89 */
            new SwerveDriveCommands(0, 0, 0.3).withTimeout(1.2),/*Original:1.2 */

            new SwerveDriveCommands(0.19, 0.2,0).withTimeout(2.3),/*Original:2.3 */
            new SwerveDriveCommands(0.19, 0.2,0).withTimeout(0.4),/*Original:0.4 */
            new SwerveDriveCommands(0,0, 0.2).withTimeout(0.4),/*Original:0.4 */
            new SwerveDriveCommands(0,0,0).withTimeout(1)
            
        );
    }
}