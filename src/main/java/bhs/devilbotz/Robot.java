// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import bhs.devilbotz.lib.AutonomousModes;
import bhs.devilbotz.subsystems.Gripper;
import bhs.devilbotz.utils.ShuffleboardManager;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.file.Files;
import java.nio.file.Paths;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private AutonomousModes autoMode;

  private ShuffleboardManager shuffleboardManager;
  private RobotContainer robotContainer;
  private static JsonNode robotConfig;

  /**
   * We default to using the "Competition BOT" robot ID if the current ID is not found. This is
   * useful for simulation
   */
  private static final String robotUniqueIdDefault =
      "00802F17DEE0"; // This is the competition bot's unique ID (MAC address)

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    try {
      String robotConfigPath = getRobotConfigFilePath();
      File robotConfigFile = new File(robotConfigPath);
      ObjectMapper objectMapper = new ObjectMapper();
      robotConfig = objectMapper.readTree(robotConfigFile);
      System.out.println("Loaded robot config: " + robotConfigPath);
      System.out.println("\tname: " + robotConfig.get("name").asText());
      System.out.println("\tid: " + robotConfig.get("id").asText());
    } catch (Exception ex) {
      System.out.println(ex.toString());
    }

    robotContainer = new RobotContainer();
    shuffleboardManager = robotContainer.getShuffleboardManager();
  }

  /**
   * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics that
   * you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic methods, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    autoMode = ShuffleboardManager.autoModeChooser.getSelected();
    shuffleboardManager.updateValues();
    Pose2d p = robotContainer.driveTrain.field.getRobotPose();
    SmartDashboard.putNumber("pose_x", p.getX());
    SmartDashboard.putNumber("pose_y", p.getY());
  }

  /** This method is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.driveTrain.resetRobotPosition();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    robotContainer.driveTrain.resetRobotPosition();

    autonomousCommand = robotContainer.getAutonomousCommand(autoMode);
    Gripper.enableCompressor();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This method is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    robotContainer.driveTrain.resetRobotPosition();

    Gripper.enableCompressor();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This method is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This method is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This method is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This method is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
   * This method returns the expected JSON config for the current robot. The current robot is
   * identified by getting the MAC address. If the robot specific config is not found, it loads the
   * default config
   *
   * @return The path of the robot specific JSON config file
   * @throws Exception If a valid JSON config file isn't found
   * @see {@link #robotUniqueIdDefault}
   */
  private static String getRobotConfigFilePath() throws Exception {
    String robotConfigFilePathPrefix =
        Filesystem.getDeployDirectory() + File.separator + "robotconfig" + File.separator;
    String robotUniqueId = getMacAddress();
    String robotConfigFilePathSuffix = ".json";

    if (Files.exists(
        Paths.get(robotConfigFilePathPrefix + robotUniqueId + robotConfigFilePathSuffix))) {
      return robotConfigFilePathPrefix + robotUniqueId + robotConfigFilePathSuffix;
    } else if (Files.exists(
        Paths.get(robotConfigFilePathPrefix + robotUniqueIdDefault + robotConfigFilePathSuffix))) {
      System.out.println(
          "WARNING: robotConfig for "
              + robotUniqueId
              + " not found, using default robot "
              + robotUniqueIdDefault);
      return robotConfigFilePathPrefix + robotUniqueIdDefault + robotConfigFilePathSuffix;
    } else {
      throw new Exception(
          "Can't find a valid robot config JSON file in: " + robotConfigFilePathPrefix);
    }
  }

  /**
   * This method returns the current robot's mac address. In simulation, it returns the current
   * computer's mac address.
   *
   * @return the current robot's mac address as a HEX string
   * @author ParkerMeyers
   * @see <a
   *     href=https://github.com/BHSRobotix/OffSeason2023/blob/main/src/main/java/bhs/devilbotz/subsystems/UniqueID.java#L26>@ParkerMeyers
   *     original code</a>
   */
  private static String getMacAddress() {
    try {
      NetworkInterface network = NetworkInterface.getByInetAddress(InetAddress.getLocalHost());
      byte[] mac = network.getHardwareAddress();

      StringBuilder macString = new StringBuilder();
      for (byte m : mac) {
        macString.append(String.format("%02X", m).replace("-", ""));
      }
      return macString.toString();
      // TODO: Implement checking for the practice bot
    } catch (SocketException | UnknownHostException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * This method returns the requested robot specific system id configuration
   *
   * @return JsonNode containing the value of the requested configuration value
   */
  public static JsonNode getSysIdConstant(String name) {
    return robotConfig.get("sysid").get(name);
  }

  /**
   * This method returns the requested robot specific drive train configuration
   *
   * @return JsonNode containing the value of the requested configuration value
   */
  public static JsonNode getDriveTrainConstant(String name) {
    return robotConfig.get("drivetrain").get(name);
  }
}
