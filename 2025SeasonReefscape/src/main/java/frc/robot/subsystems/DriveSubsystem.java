// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TeleConstants;
import frc.robot.Constants.DriveConstants.MotorLocation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Drive Subsystem */
public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.k_FrontLeftDrivingCanId,
      DriveConstants.k_FrontLeftTurningCanId,
      DriveConstants.k_FrontLeftChassisAngularOffset,
      MotorLocation.FRONT_LEFT);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.k_FrontRightDrivingCanId,
      DriveConstants.k_FrontRightTurningCanId,
      DriveConstants.k_FrontRightChassisAngularOffset,
      MotorLocation.FRONT_RIGHT);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.k_RearLeftDrivingCanId,
      DriveConstants.k_RearLeftTurningCanId,
      DriveConstants.k_BackLeftChassisAngularOffset,
      MotorLocation.REAR_LEFT);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.k_RearRightDrivingCanId,
      DriveConstants.k_RearRightTurningCanId,
      DriveConstants.k_BackRightChassisAngularOffset,
      MotorLocation.REAR_RIGHT);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.k_pigeon2Id);

  private SwerveModuleState m_desiredModuleStates[] = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
  private StructArrayPublisher<SwerveModuleState> publisherDesiredStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> publisherActualStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyActualStates", SwerveModuleState.struct).publish();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.k_DriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degree)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
     RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      //Prints exceptions
      config = null;
      e.printStackTrace();
    }
    //Configure Autobuilder after constructor
     AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.04, 0, 0), // Translation PID constants
                    new PIDConstants(0.04, 0, 0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds)
    {
        SwerveModuleState[] desiredStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(chassisSpeeds);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }
    /**
     * Gets a list of Swerve Module States
     * @return A list of Swerve Module State from front left, front right, back left, back right
     */
    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(), 
            m_frontRight.getState(), 
            m_rearLeft.getState(), 
            m_rearRight.getState()
        };
    }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degree)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

        if(DebuggingConstants.k_swerveDriveDebug) {
            updateSmartDashboard();
            // updateWheelPositions();
            
            publisherDesiredStates.set(m_desiredModuleStates);
            publisherActualStates.set(getSwerveModuleState());
        
            // Updates individual wheel values
            m_frontLeft.updateSmartDashboard();
            m_frontRight.updateSmartDashboard();
            m_rearLeft.updateSmartDashboard();
            m_rearRight.updateSmartDashboard();
        }
  }

  /**
     * Updates general robot data to SmartDasboard such as heading or pose
     */
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading (Yaw)", getHeading());
        SmartDashboard.putNumber("Roll", m_gyro.getRoll().getValue().in(Units.Degree));
        SmartDashboard.putNumber("Pitch", m_gyro.getPitch().getValue().in(Units.Degree));
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    /**
     * Upddates wheel positions to help advantage scope and other tools to visualize robot
     */
    public void updateWheelPositions() {
        SwerveModuleState[] moudleStates = getSwerveModuleState(); // Current states of wheels
        
        // Physical / IRL values
        SmartDashboard.putNumberArray(
            "RealState"
            , new double[]{
                moudleStates[0].angle.getRadians(),
                moudleStates[0].speedMetersPerSecond,
                moudleStates[1].angle.getRadians(),
                moudleStates[1].speedMetersPerSecond,
                moudleStates[2].angle.getRadians(),
                moudleStates[2].speedMetersPerSecond,
                moudleStates[3].angle.getRadians(),
                moudleStates[3].speedMetersPerSecond,
            }
        );

        // Controller / Desired values
        SmartDashboard.putNumberArray(
            "DesiredState"
            , new double[]{
                m_desiredModuleStates[0].angle.getRadians(),
                m_desiredModuleStates[0].speedMetersPerSecond,
                m_desiredModuleStates[1].angle.getRadians(),
                m_desiredModuleStates[1].speedMetersPerSecond,
                m_desiredModuleStates[2].angle.getRadians(),
                m_desiredModuleStates[2].speedMetersPerSecond,
                m_desiredModuleStates[3].angle.getRadians(),
                m_desiredModuleStates[3].speedMetersPerSecond,
            }
        );

        // Phsyical / IRL Values but with Fake Speeds to help align and see deviation
        SmartDashboard.putNumberArray(
            "RealStateFakeSpeed"
            , new double[]{
                moudleStates[0].angle.getRadians(),
                5,
                moudleStates[1].angle.getRadians(),
                5,
                moudleStates[2].angle.getRadians(),
                5,
                moudleStates[3].angle.getRadians(),
                5,
            }
        );

        // Controller / Desired Values but with Fake Speeds to help align and see deviation
        SmartDashboard.putNumberArray(
            "DesiredStateFakeSpeed"
            , new double[]{
                m_desiredModuleStates[0].angle.getRadians(),
                4,
                m_desiredModuleStates[1].angle.getRadians(),
                4,
                m_desiredModuleStates[2].angle.getRadians(),
                4,
                m_desiredModuleStates[3].angle.getRadians(),
                4,
            }
        );
    }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degree)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.k_MaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.k_MaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.k_MaxAngularSpeed;

    var swerveModuleStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Units.Degree)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.k_MaxSpeedMetersPerSecond);
    m_desiredModuleStates = swerveModuleStates;
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_desiredModuleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    m_desiredModuleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    m_desiredModuleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    m_desiredModuleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

    m_frontLeft.setDesiredState(m_desiredModuleStates[0]);
    m_frontRight.setDesiredState(m_desiredModuleStates[1]);
    m_rearLeft.setDesiredState(m_desiredModuleStates[2]);
    m_rearRight.setDesiredState(m_desiredModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.k_MaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
    m_desiredModuleStates = desiredStates;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw().getValue().in(Units.Degree);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(m_gyro.getYaw().getValue());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // Degrees per second
    return m_gyro.getAngularVelocityZWorld().getValue().in(Units.DegreesPerSecond) * (DriveConstants.k_GyroReversed ? -1.0 : 1.0);
  }

  public void stopModules() {
    m_frontLeft.stopMotors();
    m_frontRight.stopMotors();
    m_rearLeft.stopMotors();
    m_frontRight.stopMotors();

    m_desiredModuleStates[0].speedMetersPerSecond = 0;
    m_desiredModuleStates[1].speedMetersPerSecond = 0;
    m_desiredModuleStates[2].speedMetersPerSecond = 0;
    m_desiredModuleStates[3].speedMetersPerSecond = 0;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.k_DriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );
  }

  /**
     * A function designed to only drive the wheels
     * @param speed Velocity (meters per second) to turn the wheels 
     */
    public void testDriveMotors(double speed)
    {
        // Limits speed to negative max speed to max speed inclusive
        speed = Math.signum(speed) * Math.min(Math.abs(speed), TeleConstants.k_MaxAngularSpeedRadiansPerSecond);

        // Sets speed
        m_frontLeft.testDriveMotors(speed);
        m_frontRight.testDriveMotors(speed);
        m_rearLeft.testDriveMotors(speed);
        m_rearRight.testDriveMotors(speed);

        // updates the speed
         m_desiredModuleStates = new SwerveModuleState[]{
            new SwerveModuleState(speed, new Rotation2d(m_frontLeft.getTurnPosition())),
            new SwerveModuleState(speed, new Rotation2d(m_frontRight.getTurnPosition())),
            new SwerveModuleState(speed, new Rotation2d(m_rearLeft.getTurnPosition())),
            new SwerveModuleState(speed, new Rotation2d(m_rearRight.getTurnPosition()))
        };
    }
    
    /**
     * A function designed to only turn the wheels
     * @param pos Position to turn wheels to
     */
    public void testTurnMotors(double pos) {
        m_frontLeft.testTurnMotors(pos);
        m_frontRight.testTurnMotors(pos);
        m_rearLeft.testTurnMotors(pos);
        m_rearRight.testTurnMotors(pos);

        // updates the rotation
        m_desiredModuleStates = new SwerveModuleState[]{
            new SwerveModuleState(m_frontLeft.getDriveVelocity(), new Rotation2d(m_frontLeft.getTurnPosition() + pos)),
            new SwerveModuleState(m_frontRight.getDriveVelocity(), new Rotation2d(m_frontRight.getTurnPosition() + pos)),
            new SwerveModuleState(m_rearLeft.getDriveVelocity(), new Rotation2d(m_rearLeft.getTurnPosition() + pos)),
            new SwerveModuleState(m_rearRight.getDriveVelocity(), new Rotation2d(m_rearRight.getTurnPosition() + pos))
        };
    }
    
    /**
     * A function designed to continously turn the wheels
     * @param wheelPos A supplier of what the newest wheel position is
     * @param turnClockwise Whether the wheel should turn clockwise or not
     */
    public void testTurnMotors(DoubleSupplier[] wheelPos, boolean turnClockwise) {
        double val = 10 * (turnClockwise ? 1 : -1);

        m_frontLeft.testTurnMotors(wheelPos[0].getAsDouble() + val);
        m_frontRight.testTurnMotors(wheelPos[1].getAsDouble() + val);
        m_rearLeft.testTurnMotors(wheelPos[2].getAsDouble() + val);
        m_rearRight.testTurnMotors(wheelPos[3].getAsDouble() + val);

        // updates the rotation
        m_desiredModuleStates = new SwerveModuleState[]{
            new SwerveModuleState(5, new Rotation2d(wheelPos[0].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[1].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[2].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[3].getAsDouble() + val))
        };
    }
    /**
     * A list of the swerve module's latest turn angle
     * @return A supplier of the wheels latest position
     */
    public DoubleSupplier[] getWheelRotationSupplier() {
        return new DoubleSupplier[]{
            () -> m_frontLeft.getTurnPosition(),
            () -> m_frontRight.getTurnPosition(),
            () -> m_rearLeft.getTurnPosition(),
            () -> m_rearRight.getTurnPosition()
        };
    }
}
