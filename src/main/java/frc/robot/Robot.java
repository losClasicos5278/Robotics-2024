// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import com.ctre.phoenix6.
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.PIDController;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  // private static WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(3);
  // private static WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(4);
  // private static WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(1);
  // private static WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(2);
  CANSparkMax rightFrontMotor;
  CANSparkMax rightBackMotor;
  CANSparkMax leftFrontMotor;
  CANSparkMax leftBackMotor;
  CANSparkMax leftClimber;
  CANSparkMax rigthClimber;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private static final XboxController controller = new XboxController(0);
  // JoystickButton soleGreenButton = new JoystickButton(controller, Button.kA.value);
  // JoystickButton soleYellowButton = new JoystickButton(controller, Button.kY.value);
  // JoystickButton soleRedButton = new JoystickButton(controller, Button.kB.value);
  // JoystickButton extendArm = new JoystickButton(controller, Button.kRightBumper.value);
  // JoystickButton detractArm = new JoystickButton(controller, Button.kLeftBumper.value);
  PIDController pid;

  
  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    System.out.println("robotInit has been called.");
    rightFrontMotor = new CANSparkMax(4, MotorType.kBrushed);
    rightBackMotor = new CANSparkMax(2, MotorType.kBrushed);
    leftFrontMotor = new CANSparkMax(3, MotorType.kBrushed);
    leftBackMotor = new CANSparkMax(5, MotorType.kBrushed);
    leftClimber = new CANSparkMax(6, MotorType.kBrushed);
    rightClimber = new CANSparkMax(7, MotorType.kBrushed);



    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);


    
    
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
    int myInt = 1;

    myInt = 2;
    CommandScheduler.getInstance().run();
    //System.out.println("robotPeriodic has been called.");
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("disabledInit has been called.");

  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.println("autonomousInit has been called.");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    System.out.println("autonomousPeriodic has been called.");
    rightFrontMotor.set(0.5);
    rightBackMotor.set(0.5);
    leftFrontMotor.set(0.5);
    leftBackMotor.set(0.5);
   
    
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
    System.out.println("teleopInit has been called.");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //System.out.println("teleopPeriodic has been called.");
    // If up or down button pressed then move arm up or down
    // Look at old code and look at drivetrain
    //

  double speed = -controller.getLeftY();
  double turn = controller.getRightX()*0.5;
  double strafe = controller.getLeftX();

  double leftFront = speed + turn + strafe;
  double rightFront = speed - turn - strafe;
  double leftRear = speed + turn - strafe;
  double rightRear = speed - turn + strafe;

  rightFrontMotor.set(rightFront);
  rightBackMotor.set(rightRear);
  leftFrontMotor.set(leftFront);
  leftBackMotor.set(leftRear);

  
// get value for controller
boolean leftBumper = controller.getLeftBumper();
boolean rightBumper = controller.getRightBumper();
int dpad = controller.getPOV(); 

double climberSpeed = 0.0;

if (dpad > 135 && dpad < 225) {
// neg speed
  climberSpeed = -0.7;
}
else if (dpad != -1){
  if (dpad > 315 || dpad < 45){
    // pos speed 
    climberSpeed = 0.7;
  }
}
else if (dpad = -1) {
  climberSpeed = 0.0;
}


  if (leftBumper === true && rigthBumper === true){
    leftClimber.set(climberSpeed);
     rightClimber.set(ClimberSpeed);
  }
  else if (leftBumper === true){
    leftClimber.set(climberSpeed);   
  }
  else if (rightBumper === true){
    rightClimber.set(climberSpeed);
  }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    System.out.println("testInit has been called.");

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //System.out.println("testPeriodic has been called.");

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    System.out.println("simulationInit has been called.");

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    //System.out.println("simulationPeriodic has been called.");

  }
}
