package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.UpdateLogs;
import frc.robot.utils.Constants;

public class Hopper extends SubsystemBase {
    private static Hopper hopper;

    // NOTE: Possibly all 3 components below will be driven by a single motor.
    // Currently implemented this way!!!
    // Two mecanums + belts driven by a CANSparkMax
    private CANSparkMax hopperSystem;
  
    // Define sensors for the hopper to count cargo
    private DigitalInput bottomSensor, topSensor;

    private static UpdateLogs updateLogs = UpdateLogs.getInstance();

    public Hopper(){
       hopperSystem = new CANSparkMax(RobotMap.MOTOR_HOPPER, MotorType.kBrushless);
       hopperSystem.setIdleMode(IdleMode.kBrake);
       hopperSystem.setSmartCurrentLimit(Constants.MAX_HOPPER_BELT_CURRENT);
       
       //bottomSensor = new DigitalInput(0);
       //topSensor = new DigitalInput(1);

    }

    public static Hopper getInstance(){
        if (hopper == null){
            hopper = new Hopper();
        }

        return hopper;
    }

    @Override
    public void periodic() {
        updateLogs.updateHopperLogData();
    }

    public void runHopper(double speed){
        hopperSystem.set(speed); //the speed input needs a multiplier
    }

    public void stopHopper(){
       runHopper(0);
    }

    public void reverseHopper(double speed){
        runHopper(-speed);
    }

    public void putSmartDashboardOverrides(){
        SmartDashboard.putNumber("OR: Hopper speed", getHopperVelocity());
    }


    //Getters
    public double getHopperVelocity(){
        return hopperSystem.get();
    }

    public boolean sensesBallBottom(){
        return bottomSensor.get();
    }

    public boolean sensesBallTop(){
        return topSensor.get();
    }
}

