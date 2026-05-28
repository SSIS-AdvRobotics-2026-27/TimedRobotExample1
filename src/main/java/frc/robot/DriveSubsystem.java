package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem {
     // --- Motors ---
  private final SparkMax m_leftLeader;
  private final SparkMax m_leftFollower;
  private final SparkMax m_rightLeader;
  private final SparkMax m_rightFollower;

  private final DifferentialDrive m_differentialDrive;

  // --- Encoders ---
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  // Constructor
  public DriveSubsystem() {
    // Create brushed motors for a KitBot-style CIM drivetrain
    m_leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    m_leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    m_rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    m_rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);


    // Left leader: invert so that positive values drive both sides forward
    SparkMaxConfig m_leftLeaderConfig = new SparkMaxConfig();
    m_leftLeaderConfig.voltageCompensation(12);
    m_leftLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    m_leftLeaderConfig.inverted(true);
    m_leftLeader.configure(m_leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right leader: not inverted
    SparkMaxConfig m_rightLeaderConfig = new SparkMaxConfig();
    m_rightLeaderConfig.voltageCompensation(12);
    m_rightLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    m_rightLeader.configure(m_rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Followers mirror their respective leaders
    SparkMaxConfig m_leftFollowerConfig = new SparkMaxConfig();
    m_leftFollowerConfig.follow(m_leftLeader);
    m_leftFollower.configure(m_leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig m_rightFollowerConfig = new SparkMaxConfig();
    m_rightFollowerConfig.follow(m_rightLeader);
    m_rightFollower.configure(m_rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Built-in encoders
    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();

    m_differentialDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  }

  /** 
   * Perform arcade drive.
   */
  public void arcadeDrive(double speed, double rot) {
    m_differentialDrive.arcadeDrive(speed, rot);
  }

  /**
   * Returns the distance traveled by the left side in meters.
   * Negated so that forward motion (left motor inverted) gives positive distance.
   */
  public double getLeftDistanceMeters() {
    return -m_leftEncoder.getPosition() * METERS_PER_ROTATION;
  }

  /**
   * Returns the distance traveled by the right side in meters.
   */
  public double getRightDistanceMeters() {
    return m_rightEncoder.getPosition() * METERS_PER_ROTATION;
  }

  /**
   * Returns the average distance traveled by both sides in meters.
   * Convenient for straight-line distance calculations in auto.
   */
  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /** Resets both drive encoders to zero. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /** 
   * Stops drivetrain motors immediately
   */
  public void stopMotor() {
    m_differentialDrive.stopMotor();
  }

}
