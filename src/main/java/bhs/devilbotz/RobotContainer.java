// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.commands.BalancePID;
import bhs.devilbotz.commands.DriveCommand;
import bhs.devilbotz.commands.DriveSetDistancePID;
import bhs.devilbotz.commands.DriveStraight;
import bhs.devilbotz.commands.DriveStraightPID;
import bhs.devilbotz.commands.auto.BalanceAuto;
import bhs.devilbotz.commands.auto.TestAuto;
import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.subsystems.DriveTrain;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.utils.ShuffleboardManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final HashMap<AutonomousModes, Command> autoCommands = new HashMap<>();

  public final DriveTrain driveTrain = new DriveTrain();

  private final Gripper gripper = new Gripper();

  private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager();

  private final Joystick joystick =
      new Joystick(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    buildAutoCommands();
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
    driveTrain.setDefaultCommand(new DriveCommand(driveTrain, joystick::getY, joystick::getX));

    // For testing
    new JoystickButton(joystick, 1).toggleOnTrue(new BalancePID(driveTrain));
  }

  private void buildAutoCommands() {
    autoCommands.put(AutonomousModes.BALANCE, new BalanceAuto(driveTrain));
    autoCommands.put(AutonomousModes.DRIVE_DISTANCE, new DriveStraight(driveTrain, 10));
    autoCommands.put(AutonomousModes.DRIVE_DISTANCE_PID, new DriveSetDistancePID(driveTrain, 5));
    autoCommands.put(
        AutonomousModes.DRIVE_STRAIGHT_DISTANCE_PID, new DriveStraightPID(driveTrain, 5));

    autoCommands.put(AutonomousModes.TEST, new TestAuto(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(AutonomousModes autoMode) {
    Command autonomousCommand = autoCommands.get(autoMode);

    if (autoMode == null) {
      System.out.println("Robot will NOT move during autonomous :/// You screwed something up");
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
}
