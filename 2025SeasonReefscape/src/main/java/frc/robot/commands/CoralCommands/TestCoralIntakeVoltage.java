package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class TestCoralIntakeVoltage extends Command {
    private final CoralSubsystem m_coralSub;
    private double m_voltage;
    private final String m_stringMessage;

    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public TestCoralIntakeVoltage(CoralSubsystem coralSub, double voltage, String message) {
        m_coralSub = coralSub;
        m_voltage = voltage;
        m_stringMessage = message;
        SmartDashboard.putNumber(m_stringMessage, m_voltage);
        addRequirements(m_coralSub);
    }    

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putString("Commanding Coral Subsystem", m_stringMessage);
    }

    // Called every time he scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_voltage = SmartDashboard.getNumber(m_stringMessage, m_voltage);
        if(Math.abs(m_voltage) > 12) {
            m_voltage = Math.signum(m_voltage) * 12;
            SmartDashboard.putNumber(m_stringMessage, m_voltage);
        }
        //Shoots the coral fast
        m_coralSub.testIntakeMotorsVoltage(m_voltage);
    } 
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Commanding Coral Subsystem", m_stringMessage);
        //Stops the coral subsystem
        m_coralSub.stopIntakeMotor();
    }

    // Returns true when the command should end. (never happens)
    @Override
    public boolean isFinished() {
        return false;
    }
}