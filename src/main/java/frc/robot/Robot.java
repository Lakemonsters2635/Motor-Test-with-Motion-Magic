
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
//import com.ctre.

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.Encoder;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
//import com.ctre.phoenix.motorcontrol.*;

//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.JoystickNT;

//import com.revrobotics.CANEncoder;

/**
 * This sample program shows how to control a motor using a joystick, In the operator control part
 * of the program the joystick is read and the value is written to the motor
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together,
 */

public class Robot extends TimedRobot {
  private static final int frontRightMotorPort = 1;
  private static final int frontLeftMotorPort = 2;
  private static final int backRightMotorPort = 3;
  private static final int backLeftMotorPort = 4;
  private static final int leftJoystickPort = 0;
  private static final int rightJoystickPort = 1;

  private double currentPosition = 0;
  private double targetPosition = 4096 / 2;
  private boolean isFinished = false; 
  //CANEncoder smallEncoder;
  
  // Use CTRE method to set motion magic cruise velocity + accel
  /*
  max sensor velocity = 25219 units / 100 ms
  - this is measured by self-test snapshot using phoenix tuner

  cruise velocity 3000 (from 2018 elevator)
  if acceleration = velocity it will take 1s to reach cruise velocity
  
  CTRE says to set v = a and change later
  */
	public static final int ACTUATOR_VELOCITY = 300; 
	public static final int ACTUATOR_ACCELERATION = 300; 

  public static double MOTION_MAGIC_P = .4; // 0.19
	public static double MOTION_MAGIC_I = 0.0; // 0
  public static double MOTION_MAGIC_D = 0.0; // 0
   /* Calculate Kf feed forward gain
    
  peak velocity = 25225 u/100 ms    
  want to run at 25% output (for now)

  estimate velocity @ 25% peak = 0.25 * 25225 
    - in reality would it in code to be at 25% then measure true velocity w/ tuner
    
  F-gain = 0.25 * 1023 / (0.25 * 25225)
  */
  
  public static double MOTION_MAGIC_F = 0.25 * 1023 / (0.25 * 25225);

  //public static double MOTION_MAGIC_F = 0.0003;
  //private TalonSRX frontRightMotor;
  private WPI_TalonSRX frontLeftMotor;
  // private TalonSRX backRightMotor;
  // private TalonSRX backLeftMotor;
  private Joystick leftJoystick;
  private JoystickNT virtualJoystick;
  //private Joystick rightJoystick;

  @Override
  public void robotInit() {
    frontLeftMotor = new WPI_TalonSRX(frontLeftMotorPort);
    // encoder = new Encoder(0, 1, false, Encoder.EncodingType.k1X);
    
    // set factory default
    frontLeftMotor.configFactoryDefault(); 

    frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    // need configSelectedFeedbackSensor line to make graph show up in Phoenix Tuner
    // use to dial in other PID values 

    frontLeftMotor.setSensorPhase(false);

    // TODO stop @ dialing kP 

    frontLeftMotor.config_kP(0, MOTION_MAGIC_P, 0);
    frontLeftMotor.config_kI(0, MOTION_MAGIC_I, 0);
    frontLeftMotor.config_kD(0, MOTION_MAGIC_D, 0);
    frontLeftMotor.config_kF(0, MOTION_MAGIC_F, 0);
    
    // configure cancoder

    //com.ctre.phoenix.sensors.
    //CANTalon foo = null;
    //frontRightMotor = new TalonSRX(frontRightMotorPort);

    //frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    //System.out.println("Sensor Position : "+frontLeftMotor.getSelectedSensorPosition());

    //SensorCollection sensors = frontLeftMotor.getSensorCollection();

    //frontLeftMotor.set(ControlMode.Position, 0);
    
    // backRightMotor = new TalonSRX(backRightMotorPort);
    // backRightMotor.follow(frontRightMotor);
    // backLeftMotor.follow(frontLeftMotor);
    leftJoystick = new Joystick(leftJoystickPort);
    virtualJoystick = new JoystickNT("vJoy");
    // rightJoystick = new Joystick(rightJoystickPort);
  }
  //taken from frc-2018
  public void encoderStart() {
		
    System.out.println("encoderStart()");
        
    // frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    //frontLeftMotor.setSelectedSensorPosition(0, 0, 0); 	

    frontLeftMotor.selectProfileSlot(0, 0);
    	
    frontLeftMotor.configMotionAcceleration(ACTUATOR_ACCELERATION, 0);
    	
    frontLeftMotor.configMotionCruiseVelocity(ACTUATOR_VELOCITY, 0);

    // frontLeftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    // frontLeftMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    // frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    // frontLeftMotor.configNominalOutputForward(0.0,0);
    // frontLeftMotor.configNominalOutputReverse(0.0,0);
    // frontLeftMotor.configClosedloopRamp(0.5, 0);
}
  
@Override 
public void disabledInit() {
  //
  //frontLeftMotor.stop
}

  @Override
  public void teleopInit() {
    // frontLeftMotor.setSelectedSensorPosition(0, 0, 0);
    frontLeftMotor.setSelectedSensorPosition(0); 
    
    isFinished = false; 

    encoderStart();
    //FHE The Eminent Franklin Howard Evans III

    //  //frontLeftMotor.setSelectedSensorPosition(0); 
    
   // double pulsePosition = frontLeftMotor.getSensorCollection().getPulseWidthPosition();
    // System.out.println("pulsePosition :"+pulsePosition);

    double initEncPos = frontLeftMotor.getSelectedSensorPosition();
    //targetPosition = initEncPos + 1024; 
    System.out.println("TeleopInit Position before: " + initEncPos);

    //double analogIn = frontLeftMotor.getSensorCollection().getAnalogIn();
    //System.out.println("analogIn position: " + analogIn); 
    
    //frontLeftMotor.getSensorCollection().setQuadraturePosition(newPosition, timeoutMs);

    System.out.println("TeleopInit Position after: " + frontLeftMotor.getSelectedSensorPosition());
   
    //frontLeftMotor.set(ControlMode.MotionMagic, targetPosition);
   
    //super.teleopInit();
  }

  @Override
  public void teleopPeriodic() {

    // double js = leftJoystick.getY();
    double js = virtualJoystick.getRawAxis(1);

    targetPosition = 4096*js;

    int kMeasuredPosHorizontal = 1023; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = frontLeftMotor.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.1;

    //frontLeftMotor.set(ControlMode.MotionMagic, targetPosition);
    frontLeftMotor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);

   //m_motor.set( m_joystick.getY());
   //frontRightMotor.set(ControlMode.PercentOutput, leftJoystick.getY());
   //frontLeftMotor.set(ControlMode.PercentOutput, leftJoystick.getY());
   //frontLeftMotor.setSelectedSensorPosition(180);
   //double position = frontLeftMotor.getSelectedSensorPosition();
   
  //*** */  double encoderPosition = encoder.getPosition();

 //frontLeftMotor.set(ControlMode.Position, newTargetPosition);

  //ErrorCode setResults = frontLeftMotor.setSelectedSensorPosition(newTargetPosition);

  //System.out.println("setResults :"+setResults);
  //frontLeftMotor.set(ControlMode.PercentOutput, zAxis);

  double delta = 200;
  double position = frontLeftMotor.getSelectedSensorPosition();
   if(Math.abs(targetPosition - position) < delta && !isFinished) {
  
    // frontLeftMotor.set(ControlMode.Position, newTargetPosition);
    System.out.println("End Position :"+position);
    // System.out.println("getPosition :"+ encoderPosition);
    isFinished = true; 
   }
  }


}
