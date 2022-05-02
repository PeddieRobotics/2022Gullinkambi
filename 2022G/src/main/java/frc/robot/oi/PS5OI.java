package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommands.ExtendArm;
import frc.robot.commands.ClimbCommands.RetractArm;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.ShootCommands.BlankCommand;
import frc.robot.commands.ShootCommands.ShootLayup;
import frc.robot.commands.ShootCommands.ShootWithLL;
import frc.robot.commands.ShootCommands.Target;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;
import frc.robot.utils.ControllerMap;
import frc.robot.utils.Constants.OIConfig;

public class PS5OI {
    private static PS5OI oi;   

    private final Joystick driverPS5Controller = new Joystick(ControllerMap.PS5_DRIVER_PORT);

    private Intake intake;

    private Drivetrain drivetrain;

    public PS5OI(){
        intake = Intake.getInstance();
        drivetrain = Drivetrain.getInstance();

        configurePS5Controller();
    }

    public static PS5OI getInstance() {
        if (oi == null) {
            oi = new PS5OI();
        }

        return oi;
    }

    public void configurePS5Controller(){
        if(Constants.OI_CONFIG == OIConfig.PS5_TEST){
            new JoystickButton(driverPS5Controller, ControllerMap.PS5_TRIANGLE).toggleWhenPressed(new ConditionalCommand(new StopIntake(), new RunIntake(), intake::getIntakeSolenoid));
            new JoystickButton(driverPS5Controller, ControllerMap.PS5_X).whenHeld(new SequentialCommandGroup(new Target(false), new ConditionalCommand(new ShootWithLL(false), new BlankCommand(), drivetrain::isLockedOnTarget)));
            new JoystickButton(driverPS5Controller, ControllerMap.PS5_SQUARE).whenHeld(new ShootLayup(false));
            new JoystickButton(driverPS5Controller, ControllerMap.PS5_R1).whenHeld(new ExtendArm()).whenReleased(new RetractArm());

        }
    }

    public double getSpeed(){
        return -driverPS5Controller.getRawAxis(ControllerMap.PS5_LEFT_STICK_Y);
    }

    public double getTurn(){
        return driverPS5Controller.getRawAxis(ControllerMap.PS5_RIGHT_STICK_X);
    }


}
