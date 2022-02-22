package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class UnjamIntake extends CommandBase {
    private Intake m_intake;
    private Hopper m_hopper;
    private Flywheel m_flywheel;

    public UnjamIntake() {
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        m_flywheel = Flywheel.getInstance();
        addRequirements(m_intake, m_hopper, m_flywheel);
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSolenoid(true);
        m_flywheel.stopFlywheel();
    }

    @Override
    public void execute() {
        m_intake.reverseIntake(SmartDashboard.getNumber("Teleop: Intake speed", Constants.INTAKE_SPEED));
        m_hopper.reverseHopper(SmartDashboard.getNumber("Teleop: Hopper speed", Constants.HOPPER_SPEED));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hopper.stopHopper();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
