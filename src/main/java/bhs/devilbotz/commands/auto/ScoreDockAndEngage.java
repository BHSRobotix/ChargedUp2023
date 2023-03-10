package bhs.devilbotz.commands.auto;

import bhs.devilbotz.Constants.DriveConstants;
import bhs.devilbotz.commands.CommandDebug;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.assist.AutoScore;
import bhs.devilbotz.commands.drivetrain.DriveStraightPID;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will:
 *
 * <ol>
 *   <li>Auto Score
 *   <li>Move Back to make clearance for the arm
 *   <li>Put the arm down
 *   <li>Dock and Engage
 * </ol>
 *
 * @see bhs.devilbotz.commands.assist.AutoScore
 * @see bhs.devilbotz.commands.drivetrain.DriveStraightPID
 * @see bhs.devilbotz.commands.auto.DockAndEngage
 */
public class ScoreDockAndEngage extends SequentialCommandGroup {
  /**
   * Creates a sequential command that implements the Mobility routine
   *
   * @param arm the Arm object
   * @param drivetrain the DriveTrain object
   * @param gripper the gripper object
   */
  public ScoreDockAndEngage(Arm arm, DriveTrain drivetrain, Gripper gripper) {
    super();

    addCommands(CommandDebug.start());
    addCommands(new AutoScore(arm, drivetrain, gripper));
    addCommands(new DriveStraightPID(drivetrain, -DriveConstants.POSITION_DRIVE_FROM_PORTAL));
    addCommands(new ArmDown(arm, gripper));
    addCommands(new DockAndEngage(drivetrain, -2));
    addCommands(CommandDebug.end());
  }
}
