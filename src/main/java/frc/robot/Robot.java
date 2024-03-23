// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CANSparkMax rightFrontMotor;
  CANSparkMax rightBackMotor;
  CANSparkMax leftFrontMotor;
  CANSparkMax leftBackMotor;
  CANSparkMax leftClimber;
  CANSparkMax rightClimber;
  double startTime;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static final XboxController controller = new XboxController(0);
  private static final XboxController actionsController = new XboxController(1);

  // JoystickButton soleGreenButton = new JoystickButton(controller,
  // Button.kA.value);
  // JoystickButton soleYellowButton = new JoystickButton(controller,
  // Button.kY.value);

  // moves dumper up
  JoystickButton soleYellowButton = new JoystickButton(controller, Button.kY.value);
  // moves dumper down
  JoystickButton soleGreenButton = new JoystickButton(actionsController, Button.kA.value);

  // JoystickButton soleRedButton = new JoystickButton(controller,
  // Button.kB.value);
  PIDController pid;

  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  private static PneumaticsControlModule pcm = new PneumaticsControlModule(0);
  // private static DoubleSolenoid soleYellow = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  // private static DoubleSolenoid soleGreen = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  private Spark lights = new Spark(0);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();

    m_robotContainer = new RobotContainer();

    System.out.println("robotInit has been called.");

    rightBackMotor = new CANSparkMax(2, MotorType.kBrushed);
    leftFrontMotor = new CANSparkMax(3, MotorType.kBrushed);
    rightFrontMotor = new CANSparkMax(4, MotorType.kBrushed);
    leftBackMotor = new CANSparkMax(5, MotorType.kBrushed);
    leftClimber = new CANSparkMax(6, MotorType.kBrushed);
    rightClimber = new CANSparkMax(7, MotorType.kBrushless);

    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // System.out.println("robotPeriodic has been called.");
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("disabledInit has been called.");

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    startTime = System.currentTimeMillis();
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
    // rightFrontMotor.set(0.2);
    // rightBackMotor.set(0.2);
    // leftFrontMotor.set(0.2);
    // leftBackMotor.set(0.2);

    // double elapseTime = System.currentTimeMillis() - startTime;
    // if (elapseTime > 3000) {
    // rightFrontMotor.set(0);
    // rightBackMotor.set(0);
    // leftFrontMotor.set(0);
    // leftBackMotor.set(0);
    // }

    lights.set(-0.99);
    autoSelection("red");

  }

  public void autoSelection(String color) {
    double elapseTime = System.currentTimeMillis() - startTime;
    // DriverStation.Alliance alliance = !DriverStation.getAlliance().isEmpty() ?
    // DriverStation.getAlliance() : DriverStation.Alliance.Red;
    Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    System.out.println(alliance);
    // int location = DriverStation.getLocation().orElse(0);

    int driveForwardEndTime = 2150;
    int turnEndTime = 1770 + driveForwardEndTime;
    int secondDriveForwardEndTime = turnEndTime + 1000;
    lights.set(-0.61);

    if (elapseTime <= driveForwardEndTime) {
      rightFrontMotor.set(0.2);
      rightBackMotor.set(0.2);
      leftFrontMotor.set(0.2);
      leftBackMotor.set(0.2);
    } else if (elapseTime > driveForwardEndTime && elapseTime < turnEndTime) {
      if (alliance == DriverStation.Alliance.Red) {
        rightFrontMotor.set(0.2);
        rightBackMotor.set(0.2);
        leftFrontMotor.set(-0.2);
        leftBackMotor.set(-0.2);
      } else {
        rightFrontMotor.set(-0.2);
        rightBackMotor.set(-0.2);
        leftFrontMotor.set(0.2);
        leftBackMotor.set(0.2);
      }
    } else if (elapseTime > turnEndTime && elapseTime < secondDriveForwardEndTime) {
      rightFrontMotor.set(0.2);
      rightBackMotor.set(0.2);
      leftFrontMotor.set(0.2);
      leftBackMotor.set(0.2);
    } else {
      rightFrontMotor.set(0.0);
      rightBackMotor.set(0.0);
      leftFrontMotor.set(0.0);
      leftBackMotor.set(0.0);
    }

  }
  // Strafing code
  // if (alliance == DriverStation.Alliance.Blue) {
  // lights.set(-0.81);

  // rightFrontMotor.set(0.4);
  // rightBackMotor.set(-0.4);
  // leftFrontMotor.set(-0.4);
  // leftBackMotor.set(0.4);

  // if (elapseTime > 3000) {
  // rightFrontMotor.set(0);
  // rightBackMotor.set(0);
  // leftFrontMotor.set(0);
  // leftBackMotor.set(0);
  // }

  // rightFrontMotor.set(0.2);
  // rightBackMotor.set(0.2);
  // leftFrontMotor.set(0.2);
  // leftBackMotor.set(0.2);
  // Timer.delay(3);
  // rightFrontMotor.set(0);
  // rightBackMotor.set(0);
  // leftFrontMotor.set(0);
  // leftBackMotor.set(0);

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
    driveMethod2();
    activateClimbers();
    activatePistons();
    lights.set(-0.99);
  }

  public void driveMethod1() {
    // method 1 for mecanum wheels
    double turn = controller.getRightX() * 0.5;
    double strafe = controller.getLeftX();
    double speed = -controller.getLeftY();

    double leftFront = speed + turn + strafe;
    double rightFront = speed - turn - strafe;
    double leftRear = speed + turn - strafe;
    double rightRear = speed - turn + strafe;

    rightFrontMotor.set(rightFront);
    rightBackMotor.set(rightRear);
    leftFrontMotor.set(leftFront);
    leftBackMotor.set(leftRear);
  }

  public void driveMethod2() {
    // method 2 for mechanum drive
    double x = controller.getLeftX();
    double y = -controller.getLeftY();
    double turn = controller.getRightX();

    double theta = Math.atan2(y, x);
    double power = Math.hypot(x, y);

    double sin = Math.sin(theta - Math.PI / 4);
    double cos = Math.cos(theta - Math.PI / 4);
    double max = Math.max(Math.abs(sin), Math.abs(cos));

    double leftFront = power * cos / max + turn;
    double rightFront = power * sin / max - turn;
    double leftRear = power * sin / max + turn;
    double rightRear = power * cos / max - turn;

    if ((power + Math.abs(turn)) > 1) {
      leftFront /= power + Math.abs(turn);
      rightFront /= power + Math.abs(turn);
      leftRear /= power + Math.abs(turn);
      rightRear /= power + Math.abs(turn);
    }

    rightFrontMotor.set(rightFront);
    rightBackMotor.set(rightRear);
    leftFrontMotor.set(leftFront);
    leftBackMotor.set(leftRear);
  }

  public void activateClimbers() {
    // climber code
    boolean leftBumper = actionsController.getLeftBumper();
    boolean rightBumper = actionsController.getRightBumper();
    int dpad = actionsController.getPOV();

    double climberSpeed = 0.0;

    if (dpad > 135 && dpad < 225) {
      // neg speed
      climberSpeed = -1;
    } else if (dpad != -1) {
      if (dpad > 315 || dpad < 45) {
        // pos speed
        climberSpeed = 1;
      }
    } else if (dpad == -1) {
      climberSpeed = 0.0;
    }

    if (leftBumper == true && rightBumper == true) {
      leftClimber.set(climberSpeed);
      rightClimber.set(climberSpeed);
    } else if (leftBumper == true) {
      leftClimber.set(climberSpeed);
    } else if (rightBumper == true) {
      rightClimber.set(climberSpeed);
    }
  }

  public void activatePistons() {
    // code for piston
    if (actionsController.getYButtonPressed() == true) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (actionsController.getAButtonPressed() == true) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    System.out.println("testInit has been called.");
    startTime = System.currentTimeMillis();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // System.out.println("testPeriodic has been called.");

    double elapseTime = System.currentTimeMillis() - startTime;
    // DriverStation.Alliance alliance = !DriverStation.getAlliance().isEmpty() ?
    // DriverStation.getAlliance() : DriverStation.Alliance.Red;
    Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    // int location = DriverStation.getLocation().orElse(0);
    if (alliance == DriverStation.Alliance.Red) {
      lights.set(-0.61);

      if (elapseTime <= 980) {
        rightFrontMotor.set(-0.2);
        rightBackMotor.set(-0.2);
        leftFrontMotor.set(-0.2);
        leftBackMotor.set(-0.2);
      } else if (elapseTime > 1000 && elapseTime < 2760) {
        rightFrontMotor.set(-0.2);
        rightBackMotor.set(-0.2);
        leftFrontMotor.set(0.2);
        leftBackMotor.set(0.2);
      } else if (elapseTime > 2770 && elapseTime < 4560) {
        rightFrontMotor.set(-0.2);
        rightBackMotor.set(-0.2);
        leftFrontMotor.set(-0.2);
        leftBackMotor.set(-0.2);
      } else {
        rightFrontMotor.set(0.0);
        rightBackMotor.set(0.0);
        leftFrontMotor.set(0.0);
        leftBackMotor.set(0.0);
      }
    }

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    System.out.println("simulationInit has been called.");

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // System.out.println("simulationPeriodic has been called.");

  }
}
