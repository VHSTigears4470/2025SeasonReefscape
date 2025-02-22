// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.Constants.CoralConstants.CORAL_ARM_STATE;
import frc.robot.commands.AlgaeCommands.RetractAlgaeArm;
import frc.robot.commands.AlgaeCommands.ShootAlgae;
import frc.robot.commands.ClimbCommands.ExtendClimbArm;
import frc.robot.commands.ClimbCommands.RetractClimbArm;
import frc.robot.commands.CommandGroups.IntakeExtendAlgae;
import frc.robot.commands.CoralCommands.ShootCoralFast;
import frc.robot.commands.CoralCommands.ShootCoralSlow;
import frc.robot.commands.CoralCommands.ToggleCoralArm;
import frc.robot.commands.KitbotCoralCommands.OutputCoral;
import frc.robot.commands.KitbotDrivetrain.ArcadeDrive;
import frc.robot.commands.KitbotDrivetrain.MoveDistance;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.KitbotCoralSubsystem;
import frc.robot.subsystems.KitbotDriveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private DriveSubsystem m_driveSub;
  private KitbotDriveSubsystem m_kitbotDriveSub;
  private CoralSubsystem m_coralSub;
  private AlgaeSubsystem m_algaeSub;
  private ClimbSubsystem m_climbSub;
  private KitbotCoralSubsystem m_kitbotcoralSub;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.k_DriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     initSubsystems();    
    // Configure the trigger bindings
    configureBindings();
  }

  private void initSubsystems() {
        if(OperatingConstants.k_usingSwerveDrive) {
                m_driveSub = new DriveSubsystem();
                m_driveSub.setDefaultCommand(new RunCommand(
                        () -> m_driveSub.drive(
                                OIConstants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisY), OIConstants.k_DriveDeadband), 
                                OIConstants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisX), OIConstants.k_DriveDeadband), 
                                OIConstants.k_driverRotAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisRot), OIConstants.k_DriveDeadband), 
                                true,
                                "Default / Field Oriented"
                        ), 
                        m_driveSub)
                );
                m_kitbotDriveSub = null;
        } else if(OperatingConstants.k_usingKitbotDrive) { // else if because we should only have one drive at once
                m_kitbotDriveSub = new KitbotDriveSubsystem();
                m_kitbotDriveSub.setDefaultCommand(
                        new ArcadeDrive(
                                m_kitbotDriveSub,
                                () -> OIConstants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisX), OIConstants.k_DriveDeadband),
                                () -> OIConstants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisY), OIConstants.k_DriveDeadband)
                        )
                );
                m_driveSub = null;
        } else {
                m_driveSub = null;
                m_kitbotDriveSub = null;
        }

        if(OperatingConstants.k_usingAlgae) {
                m_algaeSub = new AlgaeSubsystem();
        } else {
                m_algaeSub = null;
        }

        if(OperatingConstants.k_usingClimb) {
                m_climbSub = new ClimbSubsystem();
        } else {
                m_climbSub = null;
        }

        if(OperatingConstants.k_usingCoral) {
                m_coralSub = new CoralSubsystem();
        } else {
                m_coralSub = null;
        }

        if(OperatingConstants.k_usingKitbotCoral) {
            m_kitbotcoralSub = new KitbotCoralSubsystem();
      } else {
                  m_coralSub = null;
      }
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
    int preset = 1;
    switch (preset) {
            case 0: 
                    controllerPresetMain(); // Competition Configs
                    break;
            case 1:
                    controllerPresetOne(); // Swerve
                    break;
            case 2:
                        controllerPresetTwo(); // Kitbot
                        break;
            default:
                    controllerPresetMain();
                    break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        if(OperatingConstants.k_usingSwerveDrive) {
                // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                //         3, 3);
                // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                //         new Pose2d(0, 0, new Rotation2d(0)), 
                //         List.of(),
                //         new Pose2d(1, 0, new Rotation2d(0)),
                //         trajectoryConfig
                // );
                // PIDController xController = new PIDController(1, 0, 0);
                // PIDController yController = new PIDController(1, 0, 0);
                // ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, 
                //         new TrapezoidProfile.Constraints(3, 3)
                // );
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                //         trajectory,
                //         m_driveSub::getPose,
                //         DriveConstants.k_DriveKinematics,
                //         xController,
                //         yController,
                //         thetaController,
                //         m_driveSub::setModuleStates,
                //         m_driveSub
                // );
                // return new SequentialCommandGroup(
                //         new InstantCommand(
                //                 () -> m_driveSub.resetOdometry(trajectory.getInitialPose())
                //         ),
                //         swerveControllerCommand,
                //         new InstantCommand(
                //                 () -> m_driveSub.stopModules()
                //         )
                // );
                // return new PathPlannerAuto("Straight Auto");
                
                try {
                        m_driveSub.resetEncoders();
                        m_driveSub.zeroHeading();
                        PathPlannerAuto autoPath = new PathPlannerAuto("Demo Auto");
                        m_driveSub.resetOdometry(autoPath.getStartingPose());
                        
                        return autoPath;
                } catch(Exception e) {
                        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                }
        }
        return null;
  }

  // Presets
  /**
   * The default / main preset used for comps
   * Does the following: 
   *    Left Joystick : Swerve (XY Translation)
   *    Right Joystick :  Swerve (Heading)
   *    Y : Intake/Extend Algae
   *    B : Retract Algae
   *    X : Toggle Coral Arm
   *    A :  Shoot Algae
   *    LB:  Fast Shoot
   *    LT:  Slow Shoot
   *    RB: Climb Extend
   *    RT: Climb Retract
   *    D-Pad: Intake Coral
   */
 
  public void controllerPresetMain() { //subject to change (while/on true)
        //Swerve stuff goes here
        //Left Joystick
        
        //Right Joystick
        
        //Y-button
        if(OperatingConstants.k_usingAlgae){
        m_driverController.y().onTrue(new IntakeExtendAlgae(m_algaeSub));
        }
        //B-Button
        if(OperatingConstants.k_usingAlgae){
        m_driverController.b().whileTrue(new RetractAlgaeArm(m_algaeSub));
        }
        //X-Button
        if(OperatingConstants.k_usingCoral){
        m_driverController.x().onTrue(new ToggleCoralArm(m_coralSub, CORAL_ARM_STATE.BACKWARD));
        }
        //A-Button
        if(OperatingConstants.k_usingAlgae){
        m_driverController.a().onTrue(new ShootAlgae(m_algaeSub));
        }
        //LB
        if(OperatingConstants.k_usingCoral){
        m_driverController.leftBumper().onTrue(new ShootCoralFast(m_coralSub));
        }
        //LT
        if(OperatingConstants.k_usingCoral){
        m_driverController.leftTrigger().onTrue(new ShootCoralSlow(m_coralSub));
        }
        //RB
        if(OperatingConstants.k_usingClimb){
        m_driverController.rightBumper().whileTrue(new ExtendClimbArm(m_climbSub));
        }
        //RT
        if(OperatingConstants.k_usingClimb){
        m_driverController.rightTrigger().whileTrue(new RetractClimbArm(m_climbSub));
        }
        //D-Pad 
        if(OperatingConstants.k_usingCoral){
        m_driverController.povUp().whileTrue(new ShootCoralFast(m_coralSub));// TODO chack validity later
        }
  }

  /**
   * The default / main preset used for comps
   * Does the following: 
   *    Button B : Reset Swerve's Odometer 
   *    Button A : Stops All Swerve Modules
   *    Button X : Drive Forwards
   *    Button Y : Make Wheels X
   *    Right Trigger : Hold + Joystick w/o being field orientated
   *    Right Trigger : Hold + X = Drive Left 
   */
  public void controllerPresetOne() {
        if(OperatingConstants.k_usingSwerveDrive) {
                // Reset Odom
                m_driverController.b().onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(
                                        ()-> m_driveSub.zeroHeading(),
                                        m_driveSub
                                ),
                                new InstantCommand(
                                        ()-> m_driveSub.resetOdometry(new Pose2d()),
                                        m_driveSub
                                )
                        )
                );

                // Stop Button
                m_driverController.a().onTrue(
                        new InstantCommand(
                                () -> m_driveSub.stopModules(), 
                                m_driveSub
                        )
                );

                // Drives Forward
                m_driverController.x().and(m_driverController.rightTrigger().negate()).whileTrue(
                        new RunCommand(
                                () -> m_driveSub.drive(
                                        1.0, 
                                        0.0, 
                                        0.0, 
                                        false,
                                        "Drive Forwards"
                                ), 
                                m_driveSub
                        )
                );

                // Drives Forward
                m_driverController.x().and(m_driverController.rightTrigger()).whileTrue(
                        new RunCommand(
                                () -> m_driveSub.drive(
                                        0.0, 
                                        1.0, 
                                        0.0, 
                                        false,
                                        "Drive Left"
                                ), 
                                m_driveSub
                        )
                );

                // Make Wheels X
                m_driverController.y().whileTrue(
                        new RunCommand(
                                () -> m_driveSub.setX(),
                                m_driveSub
                        )
                );

                m_driverController.leftTrigger().whileTrue(
                        new RunCommand(
                                () -> m_driveSub.resetEncoders(),
                                m_driveSub
                        )      
                );

                // drive with Robot Orientation
                m_driverController.rightTrigger().whileTrue(
                        new RunCommand(
                                () -> m_driveSub.drive(
                                        OIConstants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisY), OIConstants.k_DriveDeadband), 
                                        OIConstants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisX), OIConstants.k_DriveDeadband), 
                                        OIConstants.k_driverRotAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisRot), OIConstants.k_DriveDeadband), 
                                        false,
                                        "Robot Orientated"
                                ), 
                                m_driveSub
                        )
                );
        }
  }

  /**
   * Preset for kitbot
   * Does the following: 
   *    Button A : Drive robot forwards 5 meters
   *    Button B : Output Coral (rotates output motor)
   */
  public void controllerPresetTwo() {
        //A
        if(OperatingConstants.k_usingKitbotDrive) {
                m_driverController.a().whileTrue(new MoveDistance(m_kitbotDriveSub, 5, true));
        }

        //B
        if(OperatingConstants.k_usingKitbotCoral){
                m_driverController.b().whileTrue(new OutputCoral(m_kitbotcoralSub));
        }
  }
}