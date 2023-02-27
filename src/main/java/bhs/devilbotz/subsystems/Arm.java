// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz.subsystems;

import bhs.devilbotz.Constants;
import bhs.devilbotz.Constants.ArmConstants;
import bhs.devilbotz.commands.arm.ArmDown;
import bhs.devilbotz.commands.arm.ArmMoveDistance;
import bhs.devilbotz.commands.arm.ArmStop;
import bhs.devilbotz.commands.arm.ArmToBottom;
import bhs.devilbotz.commands.arm.ArmToMiddle;
import bhs.devilbotz.commands.arm.ArmToTop;
import bhs.devilbotz.commands.arm.ArmUp;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

/**
 * This subsystem controls arm.
 *
 * <p>The arm must start matches fully inside the robot chassis. The arm moves up using two (gas?)
 * struts. It is lowered by a motor turning a winch. A rope winds around the winch pulling the arm
 * down. In this design it is not possible to move the arm too far up and break it. But it is
 * possible for the rope around the winch to unwind completely and start winding itself again in the
 * opposite direction. This will result in 'up' commands moving the arm down (and vice versa). It is
 * also possible for the rope to become tangled around the winch/shaft if there is enough slack or
 * extra rope.
 *
 * <p>It is possible to break the arm if it is lowered too far. The motor will be winding up the
 * rope and pulling the arm down into the chassis. The arm will hit a hard stop and the motor will
 * be pulling the arm against this hard stop. It is important therefore to detect when the arm
 * reaches top most position bottom most position. We will use limit switches to do this.
 *
 * <p>There will also be an encoder mounted on the shoulder joint. Using an encoder on the shoulder
 * joint instead of on the motor or winch shaft should provide more accurate measurement of the arm
 * position. Measuring the rotations of the winch might be less accurate because the rope is winding
 * around it in an inconsistent manner. Maybe this is neglible?
 *
 * <p>There are methods to move the arm up and down at a set speed. We do not intend to vary the
 * speed of the arm or expose speed control outside the arm subsystem. There are methods to detect
 * when arm reaches the top and bottom most positions. The intent of these methods is to use them in
 * arm Commands to move the arm up and down but stop when the top and bottom most positions are
 * reached.
 *
 * <p>The encoder is not needed to do any control of the arm moving up or down. It can be used to
 * move the arm to known positions/heights. For example if we plan on picking up pieces from the
 * portal in the substation we should provide a Command that can move the arm to the correct height
 * to pickup cone/cube. We will need to determine what distance/position encoder reads to be at
 * correct height.
 *
 * <p>There is a method to detect a 'middle' position, move a set distance, and to get the current
 * position. The idea here is to create a set of commands that can move the arm to a set position
 * just high enough above a low/mid/high grid scoring node to score a cube/cone. Then provide a
 * second command that can lower the arm and release the gripper to place/score the cube/cone. Not
 * sure this will be useful to the driver. Will need to experiment with this if time allows.
 *
 * <p>There are two limit switches, one at top of range of motion and one at bottom of range of
 * motion. The limit switches are plugged into the roborio digital input/output ports.
 *
 * <p>There is a quadrature encoder on the arm joint. The signal and power lines are plugged into
 * the roborio digital input/output ports.
 */
public class Arm extends SubsystemBase {
  private final CANSparkMax armMotor;
  private final DigitalInput topLimitSwitch;
  private final DigitalInput bottomLimitSwitch;
  private final Encoder encoder;

  // These values are used to move the arm to known positions.  0 is considered starting
  // position/bottom. Values increase as arm moves up.
  //
  // TODO: The numbers need to be tuned.
  private final double POSITION_TOP = 558;
  private final double POSITION_MIDDLE = 468;
  private final double POSITION_BOTTOM = 258;
  private final double POSITION_PORTAL = 465;

  /*
   * low cone: 258
   * middle cone: 468
   * high cone: 558
   *
   * cube
   * low: 180
   * middle: 354
   * high: 440
   *
   */

  private double topPosition = POSITION_TOP;
  private double middlePosition = POSITION_MIDDLE;
  private double bottomPosition = POSITION_BOTTOM;
  private double portalPosition = POSITION_PORTAL;

  // When trying to reach a set position, how close is good enough? This value is used to determine
  // that. Smaller value tries to reach closer to target position.  Larger will stop arm further
  // away from exact position read by encoder.  Why do we need this? The arm is moving so trying to
  // stop at exact position will result in some overshoot.
  private final double positionError = 2;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("Arm");

  private DoubleEntry ntTopPosition = table.getDoubleTopic("position/top").getEntry(POSITION_TOP);
  private DoubleEntry ntMiddlePosition =
      table.getDoubleTopic("position/middle").getEntry(POSITION_MIDDLE);
  private DoubleEntry ntBottomPosition =
      table.getDoubleTopic("position/bottom").getEntry(POSITION_BOTTOM);
  private DoubleEntry ntPortalPosition =
      table.getDoubleTopic("position/portal").getEntry(POSITION_PORTAL);
  private DoubleEntry ntCurrentPosition =
      table.getDoubleTopic("position/_current").getEntry(POSITION_PORTAL);

  private BooleanEntry ntTopLimitSwitch = table.getBooleanTopic("limit/top").getEntry(false);
  private BooleanEntry ntBottomLimitSwitch = table.getBooleanTopic("limit/bottom").getEntry(false);

  private BooleanEntry ntTop = table.getBooleanTopic("state/atTop").getEntry(false);
  private BooleanEntry ntMiddle = table.getBooleanTopic("state/atMiddle").getEntry(false);
  private BooleanEntry ntBottom = table.getBooleanTopic("state/atBottom").getEntry(false);
  private BooleanEntry ntPortal = table.getBooleanTopic("state/atPortal").getEntry(false);
  private BooleanEntry ntMoving = table.getBooleanTopic("state/moving").getEntry(false);

  /** The constructor for the arm subsystem. */
  public Arm() {
    ntTopPosition.setDefault(POSITION_TOP);
    ntMiddlePosition.setDefault(POSITION_MIDDLE);
    ntBottomPosition.setDefault(POSITION_BOTTOM);
    ntPortalPosition.setDefault(POSITION_PORTAL);

    armMotor =
        new CANSparkMax(
            Constants.ArmConstants.ARM_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    topLimitSwitch = new DigitalInput(ArmConstants.TOP_LIMIT_SWITCH_DIO_PORT);
    bottomLimitSwitch = new DigitalInput(ArmConstants.BOTTOM_LIMIT_SWITCH_DIO_PORT);
    encoder =
        new Encoder(
            ArmConstants.ENCODER_CHANNEL_A_DIO_PORT, ArmConstants.ENCODER_CHANNEL_B_DIO_PORT);

    // Arm starts down inside chassis. This means encoder starts at zero here.  When moving the arm
    // up it is nice to see positive numbers for position.  With this set to
    // true moving arm up from initial position should return positive value.
    //
    // TODO: check if switching DIO port wires will change sign of value.
    //
    encoder.setReverseDirection(true);

    buildShuffleboardTab();
  }

  /**
   * This method updates once per loop of the robot.
   *
   * @see <a href="https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html">Command
   *     Based Programming</a>
   */
  @Override
  public void periodic() {
    ntTopLimitSwitch.set(isTopLimit());
    ntBottomLimitSwitch.set(isBottomLimit());
    ntTop.set(atTop());
    ntMiddle.set(atMiddle());
    ntBottom.set(atBottom());
    ntPortal.set(atSubstationPortal());
    ntCurrentPosition.set(getPosition());
    ntMoving.set(isMoving());

    topPosition = ntTopPosition.get();
    middlePosition = ntBottomPosition.get();
    bottomPosition = ntBottomPosition.get();
    portalPosition = ntPortalPosition.get();

    // System.out.println("test ntmiddle" + ntMiddle.getTopic().getName());

    // Only reset encoder position if arm hit bottom limit switch and is not moving.  The arm will
    // be engaged with limit switch while moving.  We don't want to reset encoder multiple times.
    if (isBottomLimit() && isMoving() == false) {
      resetPosition();
    }
  }

  /**
   * This method sets the speed of the arm.
   *
   * @param speed The speed of the arm.
   */
  public void setSpeed(double speed) {
    armMotor.set(speed);
  }

  /** Move the arm up at set speed. There is no check/protection against moving arm too far up. */
  public void up() {
    armMotor.set(1);
  }

  /**
   * Move the arm down at set speed. There is no check/protection against moving arm too far down.
   */
  public void down() {
    armMotor.set(-1);
  }

  /** This method stops the arm. */
  public void stop() {
    armMotor.set(0.0);
    armMotor.stopMotor();
  }

  /**
   * Check if arm is moving or not based on motor state.
   *
   * @return true if arm is moving, false if not.
   */
  public boolean isMoving() {
    return (armMotor.get() != 0.0);
  }

  /**
   * Return state of top limit switch
   *
   * @return true limit switch is pressed, false if not.
   */
  public boolean isTopLimit() {
    return !topLimitSwitch.get();
  }

  /**
   * Return state of bottom limit switch
   *
   * @return true limit switch is pressed, false if not.
   */
  public boolean isBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  /**
   * Check if arm is at bottom position to allow it to score a game piece in lower grid scoring
   * node.
   *
   * @return true if at bottom, false if not.
   */
  public boolean atBottom() {
    // Don't check for exact position, check if position is in some range.
    return (getPosition() >= bottomPosition - positionError
        && getPosition() <= bottomPosition + positionError);
  }

  /**
   * Check if arm is at top position to allow it to score a game piece in top grid scoring node.
   *
   * @return true if at top, false if not.
   */
  public boolean atTop() {
    // Don't check for exact position, check if position is in some range.
    return (getPosition() >= topPosition - positionError
        && getPosition() <= topPosition + positionError);
  }

  /**
   * Check if arm is at middle position based on encoder position. Middle is just high enough above
   * middle grid scoring node to allow cone or cube to be lined up.
   *
   * @return true if at middle position false if not.
   */
  public boolean atMiddle() {
    // Don't check for exact position, check if position is in some range.
    return (getPosition() >= middlePosition - positionError
        && getPosition() <= middlePosition + positionError);
  }

  /**
   * Check if arm is above middle position based on encoder position.
   *
   * @return true if above middle position false if not.
   */
  public boolean aboveMiddle() {
    return (getPosition() > middlePosition);
  }

  /**
   * Check if arm is below middle position based on encoder position.
   *
   * @return true if below middle position false if not.
   */
  public boolean belowMiddle() {
    return (getPosition() < middlePosition);
  }

  /**
   * Check if arm is at correct height to grab cone/cube from substation portal.
   *
   * @return true if at substation portal height, false if not.
   */
  public boolean atSubstationPortal() {
    // Don't check for exact position, check if position is in some range.
    return (getPosition() >= portalPosition - positionError
        && getPosition() <= portalPosition + positionError);
  }

  /**
   * Return current position of arm.
   *
   * @return position of arm.
   */
  public double getPosition() {
    return encoder.getDistance();
  }

  /**
   * Reset encoder to zero. We consider arm down inside chassis as zero position. Use lower limit
   * switch to determine when we reach that position. This should be called when arm is not moving.
   */
  private void resetPosition() {
    encoder.reset();
  }

  public void buildShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    ShuffleboardContainer cmdList =
        tab.getLayout("ArmCmds", BuiltInLayouts.kGrid)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

    cmdList.add(new ArmStop(this)).withPosition(0, 0);
    cmdList.add(new ArmUp(this)).withPosition(0, 1);
    cmdList.add(new ArmDown(this)).withPosition(0, 2);
    cmdList.add(new ArmMoveDistance(this, -10)).withPosition(0, 3);

    cmdList.add(new ArmToTop(this)).withPosition(1, 0);
    cmdList.add(new ArmToMiddle(this)).withPosition(1, 1);
    cmdList.add(new ArmToBottom(this)).withPosition(1, 2);

    tab.add("Arm subsystem", this).withPosition(2, 0);
    tab.add(this.encoder).withPosition(2, 1);

    ShuffleboardContainer limitsList =
        tab.getLayout("Limit Switches", BuiltInLayouts.kGrid)
            .withPosition(2, 2)
            .withSize(2, 2)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 2));

    limitsList.add("top dio " + topLimitSwitch.getChannel(), topLimitSwitch);
    limitsList.add("bottom dio " + bottomLimitSwitch.getChannel(), bottomLimitSwitch);

    ShuffleboardContainer list =
        tab.getLayout("Position", BuiltInLayouts.kGrid)
            .withPosition(4, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

    list.addBoolean("atTop", () -> atTop())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 0);

    list.addBoolean("atPortal", () -> atSubstationPortal())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1);

    list.addBoolean("atMiddle", () -> atMiddle())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 2);

    list.addBoolean("atBottom", () -> atBottom())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 3);

    list.addBoolean("aboveMiddle", () -> aboveMiddle())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(1, 1);

    list.addBoolean("belowMiddle", () -> belowMiddle())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(1, 2);

    list.addBoolean("isMoving", () -> isMoving())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(1, 3);
  }
}
