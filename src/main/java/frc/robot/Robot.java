// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  //not actually a spark
  TalonSRX driveLeftSpark = new TalonSRX(1);
  TalonSRX driveRightSpark = new TalonSRX(2);
  TalonSRX driveLeftVictor = new TalonSRX(3);
  TalonSRX driveRightVictor = new TalonSRX(4);
  RelativeEncoder intakeEncoder;

  //leds
  AddressableLED leds;
  AddressableLEDBuffer ledsBuffer;
  boolean enabled = false;
  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  TalonSRX arm = new TalonSRX(5);
  CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  XboxController driveController = new XboxController(0);
  XboxController manipulatorController = new XboxController(1);
  XboxController everythingController = new XboxController(2);
  boolean isControllerConnected = false;

  double rumbleBuffer = 0.0;

  Accelerometer accel = new BuiltInAccelerometer();

  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 1.0;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 4.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.25;

  double startTime = -1.0;
  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSpark.setInverted(false);
    driveLeftVictor.setInverted(false);
    driveRightSpark.setInverted(false);
    driveRightVictor.setInverted(false);

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     *
     * */
    arm.setInverted(true);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);
    
    intakeEncoder = intake.getEncoder();

    leds = new AddressableLED(1);
    ledsBuffer = new AddressableLEDBuffer(29);
    leds.setLength(ledsBuffer.getLength());
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;
    right = right * 1.08;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    driveLeftSpark.set(ControlMode.PercentOutput,left);
    driveLeftVictor.set(ControlMode.PercentOutput, left);
    driveRightSpark.set(ControlMode.PercentOutput, right);
    driveRightVictor.set(ControlMode.PercentOutput, right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.set(ControlMode.PercentOutput, percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    //SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    //SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    //SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    //SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  public void setLedColor(int R, int G, int B){
    for(int i=0; i<29; i++){
      ledsBuffer.setRGB(i, R, G, B);
    }
  }

  public void blinkLed(double speed, int r, int g, int b){
    if ((int)(Timer.getFPGATimestamp()*speed)%2 == 1){
      setLedColor(r, g, b);
    } else {
      setLedColor(0,0,0);
    }
  }

  public void intakeLed(double speed, int r, int g, int b){
    for(int i=0; i<13; i++){
      if(Math.abs((i-(int)(Timer.getFPGATimestamp()*speed))%6) < 3) {
        ledsBuffer.setRGB(i, r, g, b);
      } else {
        ledsBuffer.setRGB(i, 0, 0, 0);
      }
    }
    for(int i=13; i<29; i++){
      if(Math.abs((i+(int)(Timer.getFPGATimestamp()*speed))%6) < 3) {
        ledsBuffer.setRGB(i, r, g, b);
      } else {
        ledsBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  public double vectorLength(double x, double y, double z){
    return Math.sqrt((x*x)+(y*y)+(z*z));
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    //set timer for when robot is enabled
    if (startTime == -1.0 && enabled){
      startTime = Timer.getFPGATimestamp();
    }
    if(!enabled){
      startTime=-1.0;
    }

    //set controller if connected
    if(everythingController.isConnected() && !isControllerConnected){
      driveController = everythingController;
      manipulatorController = everythingController;
      isControllerConnected = true;
      System.out.println("Controller 3 connected");
    }

    if (!everythingController.isConnected() && isControllerConnected){
      driveController = new XboxController(0);
      manipulatorController = new XboxController(1);
      isControllerConnected = false;
      System.out.println("Controllers reset to default");
    }

    //controller rumble
    manipulatorController.setRumble(GenericHID.RumbleType.kBothRumble, rumbleBuffer);

    //flash led for a small amount of time after robot is enabled
    if(startTime != -1.0 && (Timer.getFPGATimestamp()-startTime) < 2.0){
      blinkLed(10.0, 255, 0, 0);
    }
    if (!enabled){
      setLedColor(255, 0, 0);
    }
    leds.setData(ledsBuffer);
    leds.start();
    enabled = false;

    //default color
    setLedColor(64, 64, 64);

  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {
    driveLeftSpark.setNeutralMode(NeutralMode.Brake);
    driveLeftVictor.setNeutralMode(NeutralMode.Brake);
    driveRightSpark.setNeutralMode(NeutralMode.Brake);
    driveRightVictor.setNeutralMode(NeutralMode.Brake);

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    // if (m_autoSelected == kConeAuto) {
    //   autonomousIntakePower = INTAKE_OUTPUT_POWER;
    // } else if (kCubeAuto == kCubeAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    //}

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    enabled = true;
    // if (m_autoSelected == kNothingAuto) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    //   return;
    // }

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if (timeElapsed < ARM_EXTEND_TIME_S){
      //arm on, intake off, no drive
      setArmMotor(ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }
    else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
      //arm off, intake on, drive off
      setArmMotor(ARM_OUTPUT_POWER);
      setIntakeMotor(-1.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
      //arm in, intake off, no drive
      setArmMotor(-ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      //arm off, intake on, drive on
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, -AUTO_DRIVE_SPEED);
    } else {
      //arm off, intake off, drive off
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    driveLeftSpark.setNeutralMode(NeutralMode.Coast);
    driveLeftVictor.setNeutralMode(NeutralMode.Coast);
    driveRightSpark.setNeutralMode(NeutralMode.Coast);
    driveRightVictor.setNeutralMode(NeutralMode.Coast);

    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {
    enabled = true;
    double armPower;
    rumbleBuffer = 0.0;
    if (manipulatorController.getRightBumper()) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
    } else if (manipulatorController.getLeftBumper()) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }

    setArmMotor(armPower);
    
    double intakePower;
    int intakeAmps;
    if (manipulatorController.getAButton()) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
      intakeLed(20.0, 153, 0, 255);
      if(Math.abs(intakeEncoder.getVelocity()) < 120.0){
        rumbleBuffer = 1.0;
      }
    } else if (manipulatorController.getXButton()) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
      intakeLed(20.0, 255, 204, 0);
      //haptic feedback
      if(Math.abs(intakeEncoder.getVelocity()) < 120.0){
        rumbleBuffer = 1.0;
      }
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
      blinkLed(3.0, 153, 0, 255);
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
      blinkLed(3.0, 255, 204, 0);
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }

    setIntakeMotor(intakePower, intakeAmps);

    if (Math.abs(intakeEncoder.getVelocity())  < 30.0
            && lastGamePiece != NOTHING
            && !(manipulatorController.getXButton())
            && !(manipulatorController.getAButton()))
    {
      setLedColor(0, 255, 0);
    }
    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returnsu a negative when we want it positive.
     */
    if (driveController.getBButton()){
      setDriveMotors(0.0, -0.3);
    }else if (driveController.getYButton()) {
      setDriveMotors(0.0, -0.7);
    } else {
      setDriveMotors(driveController.getLeftX()/2.2, driveController.getLeftY()/1.5);
    }

    if (vectorLength(accel.getX(), accel.getY(), 0.0) > 0.45) {
      driveController.setRumble(GenericHID.RumbleType.kBothRumble, vectorLength(accel.getX(), accel.getY(), 0.0));
    } else{
      driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
  }
}
