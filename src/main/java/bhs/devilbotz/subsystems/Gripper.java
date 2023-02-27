// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import bhs.devilbotz.commands.gripper.GripperClose;
import bhs.devilbotz.commands.gripper.GripperOpen;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

/**
 * This subsystem controls the gripper.
 *
 * @since 1/25/2023
 * @author joshuamanoj &amp; ParkerMeyers
 */
public class Gripper extends SubsystemBase {
  private static final PneumaticHub pneumaticHub =
      new PneumaticHub(Constants.GripperConstants.COMPRESSOR_CAN_ID);

  private final DoubleSolenoid doubleSolenoid =
      pneumaticHub.makeDoubleSolenoid(
          Constants.GripperConstants.GRIPPER_SOLENOID_FORWARD,
          Constants.GripperConstants.GRIPPER_SOLENOID_REVERSE);

  /** The constructor for the gripper subsystem. */
  public Gripper() {
    pneumaticHub.disableCompressor();

    buildShuffleboardTab();
  }

  /** This method opens the gripper. */
  public void open() {
    doubleSolenoid.set(Value.kForward);
  }

  /** This method closes the gripper. */
  public void close() {
    doubleSolenoid.set(Value.kReverse);
  }

  /** This method sets the grippers speed to 0. */
  public void stop() {
    doubleSolenoid.set(Value.kOff);
  }

  /**
   * This method updates once per loop of the robot.
   *
   * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
   *     Based Programming</a>
   */
  @Override
  public void periodic() {}

  /** Enables the compressor for the pnuematic gripper. Remains on until the robot is disabled. */
  public static void enableCompressor() {
    if (!pneumaticHub.getCompressor()) {
      pneumaticHub.enableCompressorDigital();
    }
  }

  /**
   * Returns true if the compressor pressure has reached the set value. The set value is controlled
   * physically on the robot
   *
   * @return true if pressue is at pre-configured set point
   */
  public static boolean getAtSetpoint() {
    return pneumaticHub.getPressureSwitch();
  }

  public void buildShuffleboardTab() {

    ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    tab.add("Gripper subsystem", this).withPosition(6, 0);

    ShuffleboardContainer cmdList =
        tab.getLayout("GripCmds", BuiltInLayouts.kGrid)
            .withPosition(6, 1)
            .withSize(2, 1)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 1));

    cmdList.add(new GripperOpen(this)).withPosition(0, 0);
    cmdList.add(new GripperClose(this)).withPosition(1, 0);

    ShuffleboardContainer state =
        tab.getLayout("GripState", BuiltInLayouts.kGrid)
            .withPosition(6, 2)
            .withSize(2, 2)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 1));

    state.add(doubleSolenoid);
  }
}
