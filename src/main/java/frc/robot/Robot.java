// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//test comment for 2025 code

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
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
  RelativeEncoder rightEncoder;
  RelativeEncoder leftEncoder;
  UsbCamera camera1;
  UsbCamera camera2;

  double startTime;
  boolean alreadyFired;
  boolean retractedDumper;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static final XboxController controller = new XboxController(0);
  private static final XboxController actionsController = new XboxController(1);

  // JoystickButton soleGreenButton = new JoystickButton(controller,
  // Button.kA.value);
  //
  JoystickButton leftTrigger = new JoystickButton(actionsController, Axis.kLeftTrigger.value);
  JoystickButton rightTrigger = new JoystickButton(actionsController, Axis.kRightTrigger.value);

  // If triggers are also pressed, this will allow climbers to move up and down

  // moves dumper up
  JoystickButton soleYellowButton = new JoystickButton(controller, Button.kY.value);
  // moves dumper down
  JoystickButton soleGreenButton = new JoystickButton(actionsController, Button.kA.value);

  JoystickButton soleRedButton = new JoystickButton(controller, Button.kB.value);
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
    camera1 = CameraServer.startAutomaticCapture(0);

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // camera2 = CameraServer.startAutomaticCapture(1);

    // camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    m_robotContainer = new RobotContainer();

    System.out.println("robotInit has been called.");

    rightBackMotor = new CANSparkMax(2, MotorType.kBrushed);
    leftFrontMotor = new CANSparkMax(3, MotorType.kBrushed);
    rightFrontMotor = new CANSparkMax(4, MotorType.kBrushed);
    leftBackMotor = new CANSparkMax(5, MotorType.kBrushed);
    leftClimber = new CANSparkMax(6, MotorType.kBrushless);
    rightClimber = new CANSparkMax(7, MotorType.kBrushless);

    rightBackMotor.restoreFactoryDefaults();
    leftFrontMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);

    rightClimber.setInverted(true);

    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightClimber.getEncoder();
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
    alreadyFired = false;
    retractedDumper = false;
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
    autoSelection("Red");
    /*
     * write code for:
     * /Move forward about 19 inches
     * Turn in either direction
     * Move forward for about 29 inches
     * dump
     * retract/
     * If we can, write code for:
     * /Strafing away after retracting the dumper/
     * Example code:
     * /
     * public void autoQuickDrive(double elapseTime, Alliance alliance, int
     * delayTime) {
     * int driveStartTime = delayTime;
     * int driveForwardEndTime = 2150 + driveStartTime;
     * int turnEndTime = 1770 + driveForwardEndTime;
     * int secondDriveForwardEndTime = turnEndTime + 1000;
     * int scoreEndTime = secondDriveForwardEndTime + 3000;
     * int thirdDriveForwardDriveEndTime = scoreEndTime + 2000;
     * lights.set(-0.61);
     * 
     * if (elapseTime > driveStartTime && elapseTime <= driveForwardEndTime) {
     * rightFrontMotor.set(0.2);
     * rightBackMotor.set(0.2);
     * leftFrontMotor.set(0.2);
     * leftBackMotor.set(0.2);
     * } else if (elapseTime > driveForwardEndTime && elapseTime < turnEndTime) {
     * if (alliance == DriverStation.Alliance.Red) {
     * rightFrontMotor.set(0.2);
     * rightBackMotor.set(0.2);
     * leftFrontMotor.set(-0.2);
     * leftBackMotor.set(-0.2);
     * } else {
     * rightFrontMotor.set(-0.2);
     * rightBackMotor.set(-0.2);
     * leftFrontMotor.set(0.2);
     * leftBackMotor.set(0.2);
     * }
     * } else if (elapseTime > turnEndTime && elapseTime <
     * secondDriveForwardEndTime) {
     * rightFrontMotor.set(0.2);
     * rightBackMotor.set(0.2);
     * leftFrontMotor.set(0.2);
     * leftBackMotor.set(0.2);
     * } else if (elapseTime > secondDriveForwardEndTime && elapseTime <
     * scoreEndTime) {
     * if (!alreadyFired) {
     * m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
     * alreadyFired = true;
     * }
     * } else if (elapseTime > scoreEndTime && elapseTime <
     * thirdDriveForwardDriveEndTime) {
     * if (!retractedDumper) {
     * m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
     * retractedDumper = true;
     * }
     * } else {
     * rightFrontMotor.set(0.0);
     * rightBackMotor.set(0.0);
     * leftFrontMotor.set(0.0);
     * leftBackMotor.set(0.0);
     * }
     * }/
     */

  }

  public void autoSelection(String color) {
    double elapseTime = System.currentTimeMillis() - startTime;
    // DriverStation.Alliance alliance = !DriverStation.getAlliance().isEmpty() ?
    // DriverStation.getAlliance() : DriverStation.Alliance.Red;
    Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    System.out.println(alliance);
    // int location = DriverStation.getLocation().orElse(0);

    autoQuickDrive(elapseTime, alliance, 0);
    // autoLongDrive(elapseTime, alliance, 0);
    // testMe(elapseTime, alliance, 0);
    // autoStrafeRed(elapseTime, alliance, 0);
    // autoLongStrafeRed(elapseTime, alliance, 0);
  }

  public void autoQuickDrive(double elapseTime, Alliance alliance, int delayTime) {
    int driveStartTime = delayTime;
    int driveForwardEndTime = 2115 + driveStartTime;
    int turnEndTime = 1980 + driveForwardEndTime;
    int secondDriveForwardEndTime = turnEndTime + 2000;
    int scoreEndTime = secondDriveForwardEndTime + 3000;
    int reverseDriveEndTime = scoreEndTime + 2000;
    int secondTurnEndTime = reverseDriveEndTime + 1980;
    int fourthDriveEndTime = secondTurnEndTime + 1500;
    lights.set(-0.61);

    if (elapseTime > driveStartTime && elapseTime <= driveForwardEndTime) {
      rightFrontMotor.set(0.2);
      rightBackMotor.set(0.2);
      leftFrontMotor.set(0.2);
      leftBackMotor.set(0.2);
    } else if (elapseTime > driveForwardEndTime && elapseTime < turnEndTime) {
      if (alliance == DriverStation.Alliance.Red) {
        rightFrontMotor.set(-0.2);
        rightBackMotor.set(-0.2);
        leftFrontMotor.set(0.2);
        leftBackMotor.set(0.2);
      } else if (alliance == DriverStation.Alliance.Blue) {
        rightFrontMotor.set(0.2);
        rightBackMotor.set(0.2);
        leftFrontMotor.set(-0.2);
        leftBackMotor.set(-0.2);
      } else {
        rightFrontMotor.set(0);
        rightBackMotor.set(0);
        leftFrontMotor.set(0);
        leftBackMotor.set(0);
      }
    } else if (elapseTime > turnEndTime && elapseTime < secondDriveForwardEndTime) {
      rightFrontMotor.set(0.2);
      rightBackMotor.set(0.2);
      leftFrontMotor.set(0.2);
      leftBackMotor.set(0.2);
    } else if (elapseTime > secondDriveForwardEndTime && elapseTime < scoreEndTime) {
      if (!alreadyFired) {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        alreadyFired = true;
        System.out.println("Dumper should have dumped");
      }
    }  else if (elapseTime > scoreEndTime && elapseTime < reverseDriveEndTime) {
      rightFrontMotor.set(-0.1);
      rightBackMotor.set(-0.1);
      leftFrontMotor.set(-0.1);
      leftBackMotor.set(-0.1);
    } else if (elapseTime > reverseDriveEndTime && elapseTime < secondTurnEndTime) {
      if (alliance == DriverStation.Alliance.Red) {
        rightFrontMotor.set(0.2);
        rightBackMotor.set(0.2);
        leftFrontMotor.set(-0.2);
        leftBackMotor.set(-0.2);
      } else if (alliance == DriverStation.Alliance.Blue) {
        rightFrontMotor.set(-0.2);
        rightBackMotor.set(-0.2);
        leftFrontMotor.set(0.2);
        leftBackMotor.set(0.2);
      }
    } else if (elapseTime > secondTurnEndTime && elapseTime < fourthDriveEndTime) {
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

  public void autoLongDrive(double elapseTime, Alliance alliance, int delayTime) {
    autoQuickDrive(elapseTime, alliance, delayTime);

  }

  public void autoStrafeRed(double elapseTime, Alliance alliance, int delayTime) {
    int driveStartTime = delayTime;
    int strafeEndTime = 2000 + driveStartTime;
    int driveForwardEndTime = strafeEndTime + 1000;
    int scoreEndTime = driveForwardEndTime + 1000;
    int secondStrafeEndTime = scoreEndTime + 1000;
    int thirdDriveEndTime = secondStrafeEndTime + 1000;

    if (elapseTime > driveStartTime && elapseTime <= strafeEndTime) {
      rightFrontMotor.set(-0.2);
      rightBackMotor.set(0.2);
      leftFrontMotor.set(0.2);
      leftBackMotor.set(-0.2);
    } else if (elapseTime > strafeEndTime && elapseTime < driveForwardEndTime) {
      rightFrontMotor.set(0.2);
      rightBackMotor.set(0.2);
      leftFrontMotor.set(0.2);
      leftBackMotor.set(0.2);
    } else if (elapseTime > driveForwardEndTime && elapseTime < scoreEndTime) {
      if (!alreadyFired) {
        System.out.println("Dumper should fire");
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        alreadyFired = true;
      }
    } else if (elapseTime > scoreEndTime && elapseTime < secondStrafeEndTime) {
      if (!retractedDumper) {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        retractedDumper = true;
      }
    } else if (elapseTime > secondStrafeEndTime && elapseTime < thirdDriveEndTime) {
      rightFrontMotor.set(-0.2);
      rightBackMotor.set(0.2);
      leftFrontMotor.set(0.2);
      leftBackMotor.set(-0.2);
    } else {
      rightFrontMotor.set(0.0);
      rightBackMotor.set(0.0);
      leftFrontMotor.set(0.0);
      leftBackMotor.set(0.0);
    }

  }

  public void autoLongStrafeRed(double elapseTime, Alliance alliance, int delayTime) {
    autoStrafeRed(elapseTime, alliance, delayTime);
  }

  public void testMe(double elapseTime, Alliance alliance, int delayTime) {
    int driveStartTime = delayTime;
    int strafeEndTime = 500 + driveStartTime;
    int driveForwardEndTime = strafeEndTime + 500;
    int scoreEndTime = driveForwardEndTime + 500;
    lights.set(-0.81);

    if (alliance == DriverStation.Alliance.Red) {
      if (elapseTime > driveStartTime && elapseTime <= strafeEndTime) {
        rightFrontMotor.set(-0.3);
        rightBackMotor.set(0.2);
        leftFrontMotor.set(0.2);
        leftBackMotor.set(-0.3);
      } else if (elapseTime > strafeEndTime && elapseTime <= driveForwardEndTime) {
        rightFrontMotor.set(0.2);
        rightBackMotor.set(0.2);
        leftFrontMotor.set(0.2);
        leftBackMotor.set(0.2);
      } else if (elapseTime > driveForwardEndTime && elapseTime <= scoreEndTime) {
        rightFrontMotor.set(-0.3);
        rightBackMotor.set(0.2);
        leftFrontMotor.set(0.2);
        leftBackMotor.set(-0.3);
      } else {
        rightFrontMotor.set(0.0);
        rightBackMotor.set(0.0);
        leftFrontMotor.set(0.0);
        leftBackMotor.set(0.0);
      }
    } else {
      if (alliance == DriverStation.Alliance.Blue) {
        rightFrontMotor.set(0.0);
        rightBackMotor.set(0.0);
        leftFrontMotor.set(0.0);
        leftBackMotor.set(0.0);
      }
    }
  }
  // Strafing code
  // if (alliance == DriverStation.Alliance.Blue) {
  // lights.set(-0.81);

  // rightFrontMotor.set(0.4);
  // rightBackMotor.set(-0.4);
  // leftFrontMotor.set(-0.4);,
  // leftBackMotor.set(0.4);

  // if (elapseTime > 3000) {
  // rightFrontMotor.set(0);
  // rightBackMotor.set(0);
  // leftFrontMotor.set(0);
  // leftBackMotor.set(0);
  // }

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

  private boolean canMove(double climberSpeed, double position) {

    System.out.println("position: " + Double.toString(position));
    System.out.println("climberSpeed: " + Double.toString(climberSpeed));
    if (climberSpeed < 0 && position > -230) {
      return true;
    } else if (climberSpeed > 0 && position < 0) {
      return true;
    } else {
      return false;
    }
  }

  public void activateClimbers() {
    // climber code
    boolean leftBumper = controller.getLeftBumper();
    boolean rightBumper = controller.getRightBumper();
    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();

    int dpad = controller.getPOV();

    double rightEncoderPosition = rightEncoder.getPosition();
    double leftEncoderPosition = leftEncoder.getPosition();

    double climberSpeed = 0.0;

    if (dpad > 135 && dpad < 225) {
      // neg speed
      System.out.println("Down!");
      climberSpeed = -0.8;
    } else if (dpad != -1) {
      if (dpad > 315 || dpad < 45) {
        // pos speed
        climberSpeed = 0.8;
      }
    } else if (dpad == -1) {
      climberSpeed = 0.0;
    }

    // if (leftBumper == true && rightBumper == true && rightEncoderPosition > 0 &&
    // rightEncoderPosition < 230
    // && leftEncoderPosition > 0 && leftEncoderPosition < 230) {
    // leftClimber.set(climberSpeed);
    // rightClimber.set(climberSpeed);
    // } else if (leftBumper == true && leftEncoderPosition > 0 &&
    // leftEncoderPosition < 230) {
    // leftClimber.set(climberSpeed);
    // } else if (rightBumper == true && rightEncoderPosition > 0 &&
    // rightEncoderPosition < 230) {
    // rightClimber.set(climberSpeed);
    // }

    // if (leftBumper == false) {
    // leftClimber.set(0);
    // }
    // if (rightBumper == false) {
    // rightClimber.set(0);
    // }

    if (leftBumper == true && canMove(climberSpeed, leftEncoderPosition)) {
      leftClimber.set(climberSpeed);
    } else {
      leftClimber.set(0);
    }

    if (rightBumper == true && canMove(climberSpeed, rightEncoderPosition)) {
      rightClimber.set(climberSpeed);
    } else {
      rightClimber.set(0);
    }

    if (actionsController.getBButton() == true && leftTrigger > 0) {
      leftClimber.set(climberSpeed);
    }

    if (actionsController.getBButton() == true && rightTrigger > 0) {
      rightClimber.set(climberSpeed);
    }

    if (leftEncoderPosition > 230) {
      lights.set(0.61);
    }

    System.out.println(rightEncoder.getPosition());
    System.out.println(leftEncoder.getPosition());
  }

  public void activatePistons() {
    // code for piston
    if (controller.getYButtonPressed() == true) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (controller.getAButtonPressed() == true) {
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
