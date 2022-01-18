package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class Tower extends SubsystemBase{

    private CANSparkMax towerBeltUpper, towerBeltLower;
    private static Tower tower;
    //private AnalogInput topSensor0, middleSensor1, bottomSensor2, bottomSensor3;
    private DigitalInput topSensor0, middleSensor1, bottomSensor2, bottomSensor3;
    
    public static Tower getInstance(){
        if (tower == null) {
            tower = new Tower();
        }
        return tower;
    }
    
    public Tower() {
        towerBeltUpper = new CANSparkMax(RobotMap.TOWER_BELT_UPPER, MotorType.kBrushless);
        towerBeltLower = new CANSparkMax(RobotMap.TOWER_BELT_LOWER, MotorType.kBrushless);

        towerBeltUpper.setIdleMode(IdleMode.kBrake);
        towerBeltLower.setIdleMode(IdleMode.kBrake);

        towerBeltUpper.setSmartCurrentLimit(Constants.MAX_TOWERBELT_SPEED);
        towerBeltLower.setSmartCurrentLimit(Constants.MAX_TOWERBELT_SPEED);
        
        topSensor0 = new DigitalInput(0);// may be analog sensors
        middleSensor1 = new DigitalInput(1);
        bottomSensor2 = new DigitalInput(2);
        bottomSensor3 = new DigitalInput(3);
    }

    public void runLowerBelt(double speed){
        towerBeltLower.set(speed);
    }

    public void runUpperBelt(double speed){
        towerBeltUpper.set(speed);
    }

    public void runTower(double lowerSpeed, double upperSpeed) { //to run both belts at the same time 
        runLowerBelt(lowerSpeed);
        runUpperBelt(upperSpeed);
    }

    public void reverseTower(double lowerSpeed, double upperSpeed) { 
        runTower(-lowerSpeed, -upperSpeed);
    }

    public void stopTower() {
        runTower(0.0, 0.0);
    }

    //need to change to mirror this years sensor layout
    public boolean sensesBallBottom(){
        return (bottomSensor2.get() && bottomSensor3.get());
    }

    public boolean sensesBallMiddle(){
        return middleSensor1.get();
    } 

    public boolean sensesBallTop(){
        return topSensor0.get();
    } 

    /* Analog input stuff (just in case)
    public boolean sensesBallBottom(){
        return (bottomSensor2.getAverageVoltage() < 2.0);
    }

    public boolean sensesBallMiddle(){
        return (middleSensor1.getAverageVoltage() < 2.0);
    } 

    public boolean sensesBallTop(){
        return (topSensor0.getAverageVoltage() < 2.0);
    } 

    public double bottomSensorVoltage(){
        return bottomSensor2.getVoltage();
    }

    public double middleSensorVoltage(){
        return middleSensor1.getVoltage();
    } 

    public double topSensorVoltage(){
        return topSensor0.getVoltage();
    } 
    */

    public double getLowerBeltSpeed(){
        return(towerBeltLower.get());
    }

    public double getUpperBeltSpeed(){
        return(towerBeltUpper.get());
    }

    public void putSmartDashboard(){
        SmartDashboard.putNumber("Tower Lower Belt Speed", getLowerBeltSpeed());
        SmartDashboard.putNumber("Tower Upper Belt Speed", getUpperBeltSpeed());
    }
}