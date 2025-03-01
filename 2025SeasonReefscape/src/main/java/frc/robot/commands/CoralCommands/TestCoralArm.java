package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class TestCoralArm extends Command {
    private final CoralSubsystem m_coralSub;
    private final double m_speed;

    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.    
    public TestCoralArm(CoralSubsystem coralSub, double speed) {
        m_coralSub = coralSub;
        m_speed = speed;
        addRequirements(m_coralSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time he scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Shoots the coral fast
        m_coralSub.testArmMotors(m_speed);
    } 
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //Stops the coral subsystem
        m_coralSub.stopArmMotor();
    }

    // Returns true when the command should end. (never happens)
    @Override
    public boolean isFinished() {
        return false;
    }
}