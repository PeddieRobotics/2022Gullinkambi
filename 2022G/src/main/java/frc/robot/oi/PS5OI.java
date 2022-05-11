  package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommands.ExtendArm;
import frc.robot.commands.ClimbCommands.RetractArm;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.RunIntakeEndImmediately;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.commands.IntakeCommands.UnjamIntakeSecondBall;
import frc.robot.commands.IntakeCommands.UnjamIntakeWheels;
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
            new JoystickButton(driverPS5Controller, ControllerMap.PS5_L1).whenHeld(new UnjamIntakeWheels(Constants.INTAKE_SPEED));//.whenReleased(new RunIntakeEndImmediately());
            new JoystickButton(driverPS5Controller, ControllerMap.PS5_CIRCLE).whenHeld(new UnjamIntakeSecondBall(Constants.INTAKE_SPEED));

        }
    }

    public double getSpeed(){
      return -transformJoystickInputSquared(driverPS5Controller.getRawAxis(ControllerMap.PS5_LEFT_STICK_Y));
    }

    public double getTurn(){
      //SmartDashboard.putNumber("Min Turn Value", testTransformJoystickInput(driverPS5Controller.getRawAxis(ControllerMap.PS5_RIGHT_STICK_X)));

      //return transformJoystickInputSquared(driverPS5Controller.getRawAxis(ControllerMap.PS5_RIGHT_STICK_X));

      return transformTurnJoystickInput(driverPS5Controller.getRawAxis(ControllerMap.PS5_RIGHT_STICK_X), getSpeed(), 0.09, 0.05); 
    }


    private double transformJoystickInput(double joystickInput){
        if(joystickInput == 0){
          return 0;
        }
        double absJoystickInput = Math.abs(joystickInput);
        double sign = joystickInput/absJoystickInput;
        if (absJoystickInput >= 0.05 && absJoystickInput<0.9){
          return sign*(absJoystickInput-0.05)*(0.7/(0.9-0.05));
        }
        else if (absJoystickInput >=0.9){
          return sign*(0.7+(absJoystickInput-0.9)*((1-0.7)/(1-0.9)));
        }
        return 0;
    
      }

      private double transformJoystickInputSquared(double joystickInput){
          //not really squared, its to the 1.5 power hehe XD
        if(joystickInput == 0){
          return 0;
        }
        double absJoystickInput = Math.abs(joystickInput);
        double sign = joystickInput/absJoystickInput;
        if (absJoystickInput >= 0.05 && absJoystickInput<0.9){
          return sign*Math.pow((absJoystickInput-0.05)*(0.7/(0.9-0.05)), 1.5);
        }
        else if (absJoystickInput >=0.9){
          return sign*Math.pow((0.7+(absJoystickInput-0.9)*((1-0.7)/(1-0.9))), 1.5);
        }
        return 0;
    
      }

      private double transformJoystickInputSixtyPercent(double joystickInput){
          //When joystick is at 90% input, scale is so that max output is 60%
        if(joystickInput == 0){
          return 0;
        }
        double absJoystickInput = Math.abs(joystickInput);
        double sign = joystickInput/absJoystickInput;
        if (absJoystickInput >= 0.05 && absJoystickInput<0.9){
          return sign*(absJoystickInput-0.05)*(0.6/(0.9-0.05));
        }
        else if (absJoystickInput >=0.9){
          return sign*(0.6+(absJoystickInput-0.9)*((1-0.6)/(1-0.9)));
        }
        return 0;
    
      }

      private double transformTurnJoystickInput(double turnJoystickInput, double speed, double turnFF, double deadband){
        double transformedValue;
        if (turnJoystickInput==0){
          return 0;
        }
        double absSpeed = Math.abs(speed);
        double absJoystickInput = Math.abs(turnJoystickInput);
        double sign = turnJoystickInput/absJoystickInput;
        if (absJoystickInput > deadband){
          if (absSpeed >= turnFF){
            transformedValue = (absJoystickInput-deadband)*((1-turnFF)/(1-deadband));
          }
          else {
            transformedValue = (turnFF-absSpeed)+(absJoystickInput-deadband)*((1-turnFF)/(1-deadband));
          }
          if (transformedValue > 1){
            return 1*sign;
          }
          else {
            return transformedValue*sign;
          }
        }
        else{
          return 0;
        }
      }


}
