/////////////////////////////////////////////////////////////////////////////////// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Specifics that we use
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


/* 
// Limelight imports, https://docs.limelightvision.io/docs/docs-limelight/getting-started/programming
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib

LimelightHelpers.setLEDMode_PipelineControl("");
LimelightHelpers.setLEDMode_ForceBlink("")
LimelightHelpers.setCropWindow("",-1,1,-1,1);
double tx = LimelightHelpers.getTX("");
*/



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // Establishes motors to their ID from Phoenix Tuner X
  private final WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(1);
  private final WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(3);

  private final WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(2);
  private final WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(4);

  // Even though they're PWMSparkMaxs we run them through a CAN system so we use CAN. REV Hardware detects them.
  private CANSparkMax shooter1 = new CANSparkMax(7, MotorType.kBrushed);
  private CANSparkMax shooter2 = new CANSparkMax(8, MotorType.kBrushed);

  // Ignore that ID 5 is missing, the climber motor controller just happens to be 6
  private final WPI_VictorSPX climber1 = new WPI_VictorSPX(6);


  private boolean goDown = false;
  private boolean raiseArm = false;

  // This should have worked but it just didn't
  // private final DifferentialDrive drivetrain = new DifferentialDrive((double output)->{rightMotor1.set(output);rightMotor2.set(output);}, (double output)->{leftMotor1.set(output);leftMotor2.set(output);});
  
  // Establishes xbox and stick to their IDs on FRC Driver Station, make sure Logitech Extreme 3D is on port 0 and Controller Gamepad F310 is on port 1
  private final XboxController xbox = new XboxController(1);
  private final Joystick stick = new Joystick(0);

  // Yeah I don't know where this comes from but it came with the skeleton code
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
  // Keeps motors same speed
  rightMotor1.follow(rightMotor2);
  leftMotor1.follow(leftMotor2);

  m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // Drives forward in last 4 seconds
    if (Timer.getMatchTime() < 4) {
      //drivetrain.arcadeDrive(0, .5);
      drive(0.75, 0);
    } else {
      drive(0, 0);
    }

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  //private double lerpQuadratic(douuble d)


  public void drive(double forward, double steer){
    
    // Tries to remove any stick drift
    if(forward < 0.1 && forward > -.1) forward = 0;
    if(steer < 0.1 && steer > -.1) steer = 0;
    
    // Normalizes values
    double commandMagnitude = Math.sqrt((forward*forward)+(steer*steer));
    forward *= commandMagnitude;
    steer *= commandMagnitude;

    // Attempt at grouping motors but the .follow above should do it too
    double leftCommand = -(forward - steer);
    double rightCommand = (forward + steer);

    // Gives them the values to run by
    leftMotor1.set(leftCommand);
    leftMotor2.set(leftCommand);
    rightMotor1.set(rightCommand);
    rightMotor2.set(rightCommand);

    // Prints out these values so that you can keep an eye on it
    System.out.printf("L - %.4f \t| R - %.4f \t| F - %.4f \t| S - %.4f\n",
                        leftCommand,
                        rightCommand,
                        forward,
                        steer);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double forward = -stick.getY();
    double steer = (stick.getZ())*0.6;

    

    drive(forward, steer);

    //Shoot note out
    if(xbox.getYButtonPressed()){
      shooter1.set(-5);
      shooter2.set(-5);
    }else if(xbox.getYButtonReleased()){
      shooter1.set(0);
      shooter2.set(0);
    }

    //Take note in
    if(xbox.getAButtonPressed()){
      shooter1.set(1);
      shooter2.set(0.5);
    }else if(xbox.getAButtonReleased()){
      shooter1.set(0);
      shooter2.set(0);
    }

    // Climbing
    if(xbox.getXButtonPressed()) raiseArm = true;
    if(xbox.getXButtonReleased()) raiseArm = false;
    if(xbox.getBButtonPressed())  goDown = true;
    if(xbox.getBButtonReleased()) goDown = false;

    // Sets climber motors whenever raiseArm or goDown is called
    if(raiseArm){
      climber1.set(0.5);
    }else if(goDown){
      climber1.set(-0.5);
    }else{
      climber1.set(0);
    }
    


    // Limelights
    /* 
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
    */

    // Old drivetrain
    //drivetrain.arcadeDrive(stick.getZ() * 0.6, -stick.getY(), true);


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
