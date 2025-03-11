// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Configs.AlgaeConfigs;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.commands.DriveMotors;
import frc.robot.commands.AlgaeCommands.IdleAlgae;
import frc.robot.commands.AlgaeCommands.IntakeAlgae;
import frc.robot.commands.AlgaeCommands.ShootAlgae;
import frc.robot.commands.AlgaeCommands.TestAlgaeArm;
import frc.robot.commands.AlgaeCommands.TestAlgaeIntakeVoltage;
import frc.robot.commands.AutoCommands.CoralArmShootFastSecond;
import frc.robot.commands.AutoCommands.CoralArmShootSlowSecond;
import frc.robot.commands.AutoCommands.CoralArmToIntakePosition;
import frc.robot.commands.AutoCommands.CoralArmToShootPosition;
import frc.robot.commands.ClimbCommands.PullUpArm;
import frc.robot.commands.ClimbCommands.ReleaseDownArm;
import frc.robot.commands.ClimbCommands.OverrideSpedClimbArm;
import frc.robot.commands.CoralCommands.IntakeCoral;
import frc.robot.commands.CoralCommands.ShootCoralFast;
import frc.robot.commands.CoralCommands.ShootCoralSlow;
import frc.robot.commands.CoralCommands.TestCoralArmVoltage;
import frc.robot.commands.CoralCommands.TestCoralIntakeVoltage;
import frc.robot.commands.CoralCommands.TestCoralPosition;
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
import frc.robot.subsystems.TestMotorsSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private TestMotorsSubsystem m_algaeAltSub;
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
    configureSmartDashboard();
    configurePathPlanner();
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
                m_algaeSub.setDefaultCommand(new IdleAlgae(m_algaeSub));
        } else if(OperatingConstants.k_usingAlgaeAlt) {
                m_algaeAltSub = new TestMotorsSubsystem("Algae Alt Intake", AlgaeConstants.k_algaeIntakeID, AlgaeConfigs.algaeIntakeMotor, AlgaeConstants.k_upLimitSwitchID, true);
                m_algaeSub = null;
        } else {
                m_algaeSub = null;
                m_algaeAltSub = null;
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
            m_kitbotcoralSub = null;
      }
  }

  private void configurePathPlanner() {
        NamedCommands.registerCommand("CoralArmForward", new CoralArmToIntakePosition(m_coralSub));
        NamedCommands.registerCommand("CoralArmBackward", new CoralArmToShootPosition(m_coralSub));
        
        NamedCommands.registerCommand("ShootCoralFast", new CoralArmShootFastSecond(m_coralSub, 2));
        NamedCommands.registerCommand("ShootCoralSlow", new CoralArmShootSlowSecond(m_coralSub, 3));
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
    int preset = 0;
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
            case 3:
                        controllerPresetThree(); // For Debugging
                        break;
                case 4: 
                controllerPresetFour(); // For indiv debugging
                break;
            default:
                    controllerPresetMain();
                    break;
    }
  }
// Smart Dashboard input data
  private void configureSmartDashboard() {
        if(OperatingConstants.k_usingAlgae) {
               SmartDashboard.putData("Algae Arm Positive Speed", new TestAlgaeArm(m_algaeSub, 0, "Set Algae Arm Positive Speed")); 
               SmartDashboard.putData("Algae Arm Negative Speed", new TestAlgaeArm(m_algaeSub, 0, "Set Algae Arm Negative Speed")); 
               SmartDashboard.putData("Algae Intake Positive Voltage", new TestAlgaeIntakeVoltage(m_algaeSub, 0, "Set Algae Intake Positive Voltage")); 
               SmartDashboard.putData("Algae Dispense Negative Voltage", new TestAlgaeIntakeVoltage(m_algaeSub, 0, "Set Algae Intake Negative Voltage")); 
               SmartDashboard.putData("Alage Reset Encoders", new InstantCommand(()->m_algaeSub.resetEncoders(), m_algaeSub));
        }
        if(OperatingConstants.k_usingClimb) {
                SmartDashboard.putData("Climb Positive Speed", new OverrideSpedClimbArm(m_climbSub, 0.3, "Set Climb Intake Positive Speed"));
                SmartDashboard.putData("Climb Negative Speed", new OverrideSpedClimbArm(m_climbSub, -0.1, "Set Climb Intake Negative Speed"));
                SmartDashboard.putData("Climb Reset Encoders", new InstantCommand(()->m_climbSub.resetEncoders(), m_climbSub));
        }
        if(OperatingConstants.k_usingCoral) {
                SmartDashboard.putData("Coral Intake Positive Voltage", new TestCoralIntakeVoltage(m_coralSub, 2, "Set Coral Intake Positive Voltage"));
                SmartDashboard.putData("Coral Intake Negative Voltage", new TestCoralIntakeVoltage(m_coralSub, -11, "Set Coral Intake Negative Voltage"));
                SmartDashboard.putData("Coral Arm Positive Voltage", new TestCoralArmVoltage(m_coralSub,1, "Set Coral Arm Positive Voltage"));
                SmartDashboard.putData("Coral Arm Negative Voltage", new TestCoralArmVoltage(m_coralSub, -1, "Set Coral Arm Negative Voltage"));
                SmartDashboard.putData("Coral Reset Encoders", new InstantCommand(()->m_coralSub.resetEncoders(), m_coralSub));
        }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        if(OperatingConstants.k_usingSwerveDrive) {     
                // /*   
                try {
                        m_driveSub.resetEncoders();
                        m_driveSub.zeroHeading();
                        PathPlannerAuto autoPath = new PathPlannerAuto("Demo Auto");
                        m_driveSub.resetOdometry(autoPath.getStartingPose());
                        
                        return new SequentialCommandGroup(
                                new InstantCommand(
                                        () -> {
                                                System.out.println(autoPath.getStartingPose());
                                        }
                                ),
                                autoPath
                        );
                } catch(Exception e) {
                        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                }
                // */
                
                /*
                // Basic Auto That Should Move Robot in One Direction
                return new MoveDirection(m_driveSub, "Straight", new Translation2d(1, 0));
                */
        }
        return null;
  }
  
  public void onStart() {
        if(OperatingConstants.k_usingAlgae) {
                m_algaeSub.resetEncoders();
                m_algaeSub.changeArmConfig();
        }
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
        //Right Trigger
        if(OperatingConstants.k_usingAlgae){
                m_driverController.rightTrigger().whileTrue(new IntakeAlgae(m_algaeSub));
        } else if (OperatingConstants.k_usingAlgaeAlt) {
                m_driverController.rightTrigger().whileTrue(new DriveMotors(m_algaeAltSub, -.35));
        }

        //Right Bumper
        if(OperatingConstants.k_usingAlgae){
                m_driverController.rightBumper().whileTrue(new ShootAlgae(m_algaeSub));
        } else if (OperatingConstants.k_usingAlgaeAlt) {
                m_driverController.rightBumper().whileTrue(new DriveMotors(m_algaeAltSub, .35));
        }

        //Back
        if(OperatingConstants.k_usingAlgae){
                m_driverController.back().onTrue(new IdleAlgae(m_algaeSub));
        }

        //Start
        if(OperatingConstants.k_usingClimb){
                m_driverController.start().whileTrue(new PullUpArm(m_climbSub));
        }

        //Left Bumper 
        if(OperatingConstants.k_usingClimb){
                m_driverController.leftBumper().whileTrue(new ReleaseDownArm(m_climbSub));
        }

        // Left Trigger
        if(OperatingConstants.k_usingSwerveDrive){
                m_driverController.leftTrigger().whileTrue(
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

        //Y-Button
        if(OperatingConstants.k_usingCoral){
                m_driverController.y().onTrue(new ToggleCoralArm(m_coralSub));
        }

        //B-Button
        if(OperatingConstants.k_usingCoral){
                m_driverController.b().whileTrue(new ShootCoralFast(m_coralSub));
        }

        //A-Button
        if(OperatingConstants.k_usingCoral){
                m_driverController.a().whileTrue(new ShootCoralSlow(m_coralSub));
        }

        //X-Button
        if(OperatingConstants.k_usingCoral){
                m_driverController.x().whileTrue(new IntakeCoral(m_coralSub));// TODO chack validity later
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

  /**
   * NOTE : ONLY ONE SUBSYSTEM AT ONCE
   * 
   * Climb
   *  Right Trigger = Positive Speed
   *  Left Trigger = Positive Speed
   *  Y Button = Reset Encoder
   * 
   * Algae
   *  Right Trigger = Arm Positive Speed
   *  Left Trigger = Arm Negative Speed
   *  Right Bumper = Intake Positive Voltage
   *  Left Bumper = Intake Negative Voltage
   *  Y Button = Reset Encoder
   * 
   * Coral
   *  Right Trigger = Arm Positive Speed
   *  Left Trigger = Arm Negative Speed
   *  Right Bumper = Intake Positive Voltage
   *  Left Bumper = Intake Negative Voltage
   *  Y Button = Reset Encoder
   * 
   * Swerve
   * 
   * 
   */
  public void controllerPresetThree() {
        // Used For Debugging Subsystems
        if(OperatingConstants.k_usingAlgae) {
                m_driverController.rightTrigger().whileTrue(
                        new TestAlgaeArm(m_algaeSub, 0, "Set Algae Arm Positive Speed")
                );
                m_driverController.leftTrigger().whileTrue(
                        new TestAlgaeArm(m_algaeSub, 0, "Set Algae Arm Negative Speed")
                );
                m_driverController.rightBumper().whileTrue(
                        new TestAlgaeIntakeVoltage(m_algaeSub, 0, "Set Algae Intake Positive Voltage")
                );
                m_driverController.leftBumper().whileTrue(
                        new TestAlgaeIntakeVoltage(m_algaeSub, 0, "Set Algae Intake Negative Voltage")
                );
                m_driverController.y().onTrue(
                        new InstantCommand(()->m_algaeSub.resetEncoders(), m_algaeSub)
                );
         }
         if(OperatingConstants.k_usingClimb) {
                m_driverController.rightTrigger().whileTrue(
                        new OverrideSpedClimbArm(m_climbSub, 0, "Set Coral Intake Positive Speed")
                );
                m_driverController.leftTrigger().whileTrue(
                        new OverrideSpedClimbArm(m_climbSub, 0, "Set Climb Intake Negative Speed")
                );
                m_driverController.y().onTrue(
                        new InstantCommand(()->m_climbSub.resetEncoders(), m_climbSub)
                );
         }
         if(OperatingConstants.k_usingCoral) {
                m_driverController.rightTrigger().whileTrue(
                        new TestCoralArmVoltage(m_coralSub,0, "Set Coral Arm Positive Voltage")
                );
                m_driverController.leftTrigger().whileTrue(
                        new TestCoralArmVoltage(m_coralSub, 0, "Set Coral Arm Negative Voltage")
                );
                m_driverController.rightBumper().whileTrue(
                        new TestCoralIntakeVoltage(m_coralSub, 0, "Set Coral Intake Positive Voltage")
                );
                m_driverController.leftBumper().whileTrue(
                        new TestCoralIntakeVoltage(m_coralSub, 0, "Set Coral Intake Negative Voltage")
                );
                m_driverController.y().onTrue(
                        new InstantCommand(()->m_coralSub.resetEncoders(), m_coralSub)
                );
         }
  }

  public void controllerPresetFour() {
        if(OperatingConstants.k_usingAlgae) {
                m_driverController.a().whileTrue(
                        new IdleAlgae(m_algaeSub)
                );

                m_driverController.b().whileTrue(
                        new IntakeAlgae(m_algaeSub)
                );

                m_driverController.y().whileTrue(
                        new ShootAlgae(m_algaeSub)
                );
        }

        if(OperatingConstants.k_usingCoral) {
                m_driverController.a().whileTrue(
                        new TestCoralPosition(m_coralSub, 0)
                );
                
                m_driverController.b().whileTrue(
                        new TestCoralPosition(m_coralSub, -30.43)
                );

                m_driverController.x().onTrue(
                        new InstantCommand( 
                                () -> {m_coralSub.resetEncoders();}
                                , m_coralSub
                        )
                );
        }
        
        if(OperatingConstants.k_usingClimb){
                //Left Bumper
                m_driverController.start().whileTrue(new PullUpArm(m_climbSub));
                //Left Trigger 
                m_driverController.leftBumper().whileTrue(new ReleaseDownArm(m_climbSub));
        }

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
        }
  }
}