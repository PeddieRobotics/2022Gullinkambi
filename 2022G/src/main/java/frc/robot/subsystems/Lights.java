package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
// import frc.robot.commands.ClimbCommands.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Lights extends SubsystemBase{

    private static Lights lights;
    private Solenoid strip;

    public Lights() {
        strip = new Solenoid(21,PneumaticsModuleType.REVPH, 15);
    }

    public static Lights getInstance(){
        if(lights == null){
          lights = new Lights();
        }
        return lights;
      }

    public void on(){
        strip.set(true);
    }  

    public void off(){
        strip.set(false);
    }  

    @Override
    public void periodic() {
     }
}