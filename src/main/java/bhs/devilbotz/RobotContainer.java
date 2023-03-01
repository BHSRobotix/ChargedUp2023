// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.commands.DriveCommand;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmIdle;
import bhs.devilbotz.commands.arm.ArmMoveDistance;
import bhs.devilbotz.commands.arm.ArmStop;
import bhs.devilbotz.commands.arm.ArmToBottom;
import bhs.devilbotz.commands.arm.ArmToMiddle;
import bhs.devilbotz.commands.arm.ArmToTop;
import bhs.devilbotz.commands.arm.ArmUp;
import bhs.devilbotz.commands.auto.BalancePID;
import bhs.devilbotz.commands.auto.DriveStraightPID;
import bhs.devilbotz.commands.auto.DriveStraightToDock;
import bhs.devilbotz.commands.auto.RotateDegrees;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.commands.gripper.GripperIdle;
import bhs.devilbotz.commands.gripper.GripperOpen;
import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.subsystems.Arm;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.utils.ShuffleboardManager;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();

  private static final Gripper gripper = new Gripper();

  private static final Arm arm = new Arm();

  private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager();

  private final Joystick leftJoystick =
      new Joystick(Constants.OperatorConstants.DRIVER_LEFT_CONTROLLER_PORT);

  private final Joystick rightJoystick =
      new Joystick(Constants.OperatorConstants.DRIVER_RIGHT_CONTROLLER_PORT);

  // For debugging balance PID. Allows setting balance PID values on the fly
  private final PIDController balancePid =
      new PIDController(
          Robot.getDriveTrainConstant("BALANCE_P").asDouble(),
          Robot.getDriveTrainConstant("BALANCE_I").asDouble(),
          Robot.getDriveTrainConstant("BALANCE_D").asDouble());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // For debugging balance PID. Allows setting balance PID values on the fly
    SmartDashboard.putData("Balance PID", balancePid);
    SmartDashboard.putData(driveTrain);

    arm.setDefaultCommand(new ArmIdle(arm));
    gripper.setDefaultCommand(new GripperIdle(gripper));

    driveTrain.setDefaultCommand(
        new DriveCommand(driveTrain, rightJoystick::getY, rightJoystick::getX));
    // driveTrain.setDefaultCommand(new ArcadeDriveOpenLoop(driveTrain, rightJoystick::getY,
    // rightJoystick::getX));
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    new JoystickButton(leftJoystick, 1)
        .onTrue(new GripperClose(gripper))
        .onFalse(new GripperIdle(gripper));

    new JoystickButton(leftJoystick, 2)
        .onTrue(new GripperOpen(gripper))
        .onFalse(new GripperIdle(gripper));

    new JoystickButton(leftJoystick, 5).whileTrue(new ArmUp(arm)).onFalse(new ArmStop(arm));

    new JoystickButton(leftJoystick, 4)
        .whileTrue(new ArmDown(arm, gripper))
        .onFalse(new ArmStop(arm));

    new JoystickButton(leftJoystick, 6).onTrue(new ArmToTop(arm));
    new JoystickButton(leftJoystick, 7)
        .onTrue(new ArmMoveDistance(arm, -10).andThen(new GripperOpen(gripper)));

    new JoystickButton(leftJoystick, 8).onTrue(new ArmToMiddle(arm));
    new JoystickButton(leftJoystick, 9)
        .onTrue(new ArmMoveDistance(arm, -10).andThen(new GripperOpen(gripper)));

    new JoystickButton(leftJoystick, 10).onTrue(new ArmToBottom(arm));
    new JoystickButton(leftJoystick, 11)
        .onTrue(new ArmMoveDistance(arm, -10).andThen(new GripperOpen(gripper)));

    SmartDashboard.putData("gripperOpen", new GripperOpen(gripper));
    SmartDashboard.putData("gripperClose", new GripperClose(gripper));

    SmartDashboard.putData(
        "armScorePiece", new ArmMoveDistance(arm, -10).andThen(new GripperOpen(gripper)));

    /*
    new JoystickButton(leftJoystick, 6)
            .whileTrue( Cone Mode );

    new JoystickButton(leftJoystick, 7)
            .whileTrue( Cube Mode );
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @param autoMode the selected autonmous mode
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(AutonomousModes autoMode) {
    Command autonomousCommand = null;
    PathPlannerTrajectory path = null;

    if (autoMode == null) {
      System.out.println("Robot will NOT move during autonomous :/// You screwed something up");
    } else {
      switch (autoMode) {
        case SIT_STILL:
          break;
        case MOBILITY:
          autonomousCommand =
              Commands.waitSeconds(ShuffleboardManager.autoDelay.getDouble(0))
                  .asProxy()
                  .andThen(
                      new DriveStraightPID(
                          driveTrain,
                          ShuffleboardManager.autoDistance.getDouble(
                              Constants.DEFAULT_DISTANCE_MOBILITY)));
          break;
        case SCORE_AND_MOBILITY:
          break;
        case DOCK_AND_ENGAGE:
          autonomousCommand =
              Commands.waitSeconds(ShuffleboardManager.autoDelay.getDouble(0))
                  .asProxy()
                  .andThen(
                      new DriveStraightToDock(
                              driveTrain,
                              ShuffleboardManager.autoDistance.getDouble(
                                  Constants.DEFAULT_DISTANCE_DOCK_AND_ENGAGE))
                          .andThen(new BalancePID(driveTrain, balancePid)).andThen(new RotateDegrees(driveTrain, 90)));
          break;
        case MOBILITY_DOCK_AND_ENGAGE_HUMAN_SIDE:
          if (Alliance.Blue == DriverStation.getAlliance()) {
            path = PathPlanner.loadPath("MobilityBlueHumanSideToDock", new PathConstraints(2.5, 1));
          } else {
            path = PathPlanner.loadPath("MobilityRedHumanSideToDock", new PathConstraints(2.5, 1));
          }
          autonomousCommand =
              Commands.waitSeconds(ShuffleboardManager.autoDelay.getDouble(0))
                  .asProxy()
                  .andThen(driveTrain.followTrajectoryCommand(path, true))
                  .andThen(new BalancePID(driveTrain));
          break;
        case MOBILITY_DOCK_AND_ENGAGE_WALL_SIDE:
          if (Alliance.Blue == DriverStation.getAlliance()) {
            path = PathPlanner.loadPath("MobilityBlueWallSideToDock", new PathConstraints(2.5, 1));
          } else {
            path = PathPlanner.loadPath("MobilityRedWallSideToDock", new PathConstraints(2.5, 1));
          }
          autonomousCommand =
              Commands.waitSeconds(ShuffleboardManager.autoDelay.getDouble(0))
                  .asProxy()
                  .andThen(driveTrain.followTrajectoryCommand(path, true))
                  .andThen(new BalancePID(driveTrain));
          break;
        case SCORE_DOCK_AND_ENGAGE:
          break;

        case SCORE_MOBILITY_DOCK_ENGAGE:
          break;
        case SCORE_MOBILITY_PICK_DOCK_ENGAGE:
          break;
        case TEST:
          break;
        default:
          break;
      }
    }

    return autonomousCommand;
  }

  /**
   * Returns the {@link ShuffleboardManager} instance.
   *
   * @return the {@link ShuffleboardManager} instance
   */
  public ShuffleboardManager getShuffleboardManager() {
    return shuffleboardManager;
  }

  /**
   * Resets the robots position to the pose
   *
   * @see DriveTrain#resetRobotPosition()
   */
  public void resetRobotPosition() {
    driveTrain.resetRobotPosition();
  }

  /** Initialize gripper to known/same position */
  public void initGripper() {
    gripper.close();
  }

  /**
   * Create a command to move arm down. This is a workaround/hack to allow Shuffleboard to add arm
   * movement command that requires both arm and gripper. The arm Shuffleboard tab is setup in the
   * Arm subsystem which has no idea about the Gripper subsystem.
   *
   * @return command to move arm down
   */
  public static CommandBase createArmDownCommand() {
    return new ArmDown(arm, gripper);
  }
}
