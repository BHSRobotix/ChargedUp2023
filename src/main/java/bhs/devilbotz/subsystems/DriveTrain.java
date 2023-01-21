package bhs.devilbotz.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    // Define talons
    private static final WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
    private static final WPI_TalonSRX rightMaster = new WPI_TalonSRX(4);
    private static final WPI_TalonSRX leftFollower = new WPI_TalonSRX(1);
    private static final WPI_TalonSRX rightFollower = new WPI_TalonSRX(2);

    // Define NAVX
    private static final AHRS navx = new AHRS(SPI.Port.kMXP);

    // Define differential drive
    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

    private final SlewRateLimiter leftSlew = new SlewRateLimiter(5);

    private final SlewRateLimiter rightSlew = new SlewRateLimiter(5);

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain() {
        setupTalons();
        resetNavx();
        resetEncoders();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    private void setupTalons() {
        rightMaster.setInverted(true);
        leftMaster.setInverted(false);
        // Set the talons to follow each other
        rightFollower.follow(rightMaster);
        leftFollower.follow(leftMaster);

        // Set the follower talons to invert to match the master talons
        rightFollower.setInverted(InvertType.FollowMaster);
        leftFollower.setInverted(InvertType.FollowMaster);

        this.setTalonMode(NeutralMode.Brake);

        // Set the sensor phase of the master talons
        rightMaster.setSensorPhase(true);
        leftMaster.setSensorPhase(true);
    }

    /**
     * Reset the NAVX
     *
     * @since 1.0.0
     */
    public void resetNavx() {
        navx.reset();
        navx.calibrate();
    }

    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public void arcadeDrive(double speed, double rotation) {
        differentialDrive.arcadeDrive(speed, rotation);
    }

    public void setTalonMode(NeutralMode mode) {
        leftMaster.setNeutralMode(mode);
        rightMaster.setNeutralMode(mode);
        leftFollower.setNeutralMode(mode);
        rightFollower.setNeutralMode(mode);
    }

    public WPI_TalonSRX getLeftMaster() {
        return leftMaster;
    }

    public WPI_TalonSRX getRightMaster() {
        return rightMaster;
    }

    public WPI_TalonSRX getLeftFollower() {
        return leftFollower;
    }

    public WPI_TalonSRX getRightFollower() {
        return rightFollower;
    }

    public AHRS getNavx() {
        return navx;
    }
}
