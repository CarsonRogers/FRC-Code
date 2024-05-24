# Setting up Intake, Shooter, and Feeder subsystems

<details>
    <summary>Content</summary>
    <ol>
        <li><a href="#basics">The Basics</a></li>
        <li><a href="#intake">Intakes</a></li>
        <li><a href="#shooter">Shooters</a></li>
        <li><a href="#feed">Feeders</a></li>
        <li><a href="#intakeshoot">Combining the Intake and Shooter</a></li>
        <li><a href="#button">Initializing Buttons and Commands</a></li>

    </ol>
</details>

<div id="basics"></div>

# The Basics

The basic idea of any intake, shooter, or feeder subsystem can be broken down into a couple of parts.

The first part is the motors. Every intake, shooter, or feeder will have usually 1-4 motors that will run the subsystem. These motors will need initialized with all their ids and settings to make sure they run together

If the RPM wants to be tracked, then at least one RelativeEncoder will need to be created and then initialized from a motor. 

There are two ways to control the motors, set percentages, or PID controllers. 

The first way, percentages work by setting the motors to a set percentage of their output. This is useful for general intake, extake, or feed functions. 

The second way is to use a PID controller to reach a certian speed. This is mostly used with shooters that are given a certian speed to reach, then the PID controller will go and reach that speed. This can be used in conjuntion with vision processing where the distance, speed, and angle to the target can be determined then the robot auto aims and shoots at the target. 

These examples all use set percentages, but there are resources online to use PID controllers. 

The other part of any intake, shooter, or feeder is limit switchs or line breakers. 

These sensors can help detect if game pieces are shot or intaked into or out from the robot. They can be used to automate the intake/extake/shooting processes. See the intakeShooter subsystem for use of line break sensors or check out the Intake subsystem of the 2024 robot code

<div id="intake"></div>

# Intakes

This intake subsystem creates a basic one motor intake that can be expanded to multiple motors if needed. 

```
private CANSparkMax intakeMotor;

/**
* Creates a new basic Intake subsystem
* This Intake consists of one intake motor controlling the intake of a game piece
*/
public Intake() {
intakeMotor = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
configIntakeMotor();
}
```

The first part is declaring the motor, then everything is initialized in the constructor. 

```
private void configIntakeMotor(){
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(IntakeConstants.invertIntake);
    intakeMotor.setIdleMode(IntakeConstants.intakeIdleMode);
}
```

A method called configIntakeMotor configures the intake motor based off of the declared constants.

For more than one motor, one of the motors may need inverted.

Generally for intakes, feeders, or shooters the IdleMode can be set to coast as the game pieces should not have to need to come to an abrupt stop

```
/**
* An intake method that when called, calls the other intake method with the defined
* intake speed declared in constants. This method is basically acts as a 
* default for the other intake method. If no speed is declared in the other intake method, 
* then this method will run with the intake speed in constants
*/
public void intake(){
    intake(IntakeConstants.intakeSpeed);
}

/**
* Sets the percentage output of the intake motor
* @param speed the percentage output of the intake motor from 0 to 1
*/
public void intake(double speed){
    intakeMotor.set(speed);
}
```

This subsystem has two intake methods. If the motors want to be set by a single percentage speed, then a method can be used to set the motor to a specific speed declared in constants. 

Otherwise, if multiple set percentages are needed, then a general method that takes a paremter of the speed can be used. That way, the method can be called when needed and passed its appropriate speed percentage instead of always having the same set percentage.  

```
/**
* Stops the intake motor from moving.
*/
public void stop(){
    intakeMotor.set(0);
}
```

One important thing with any method is the creation of a stop method. Generally this method is set as the default method so that the intake is stopped unless otherwise stated. 

<div id="shooter"></div>

# Shooters

As with the intake subsystem. The shooter subsystem needs motors declared. Most shooters have 2 or 4 motors so that the shot object will reach an ample enough speed to make the intended distance/height to be shot. 

Along with the declaring all the motors, in order to read the RPM of the motors to determine whether the shooter should shoot or not, some encoders are needed. These can be the RelativeEncoders built into the motors and motor controllers. 

```
rightShooterMotor = new CANSparkMax(ShooterConstants.rightShooterID, MotorType.kBrushless);
leftShooterMotor = new CANSparkMax(ShooterConstants.leftShooterID, MotorType.kBrushless);
configShooterMotors();

auxRightShooterMotor = new CANSparkMax(ShooterConstants.auxRightShooterID, MotorType.kBrushless);
auxLeftShooterMotor = new CANSparkMax(ShooterConstants.auxLeftShooterID, MotorType.kBrushless);
configAuxShooterMotors();

shooterEncoder = rightShooterMotor.getEncoder();
auxShooterEncoder = auxRightShooterMotor.getEncoder();
```

Every motor and encoder needs initialized within the shooter subsystem's constructor. Here all the motors are initialized and configured.

```
  /**
   * Configures the right and left shooter motors by resetting them and 
   * setting all necessary settings declared in Constants
   */
  private void configShooterMotors(){
    rightShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.restoreFactoryDefaults();

    rightShooterMotor.enableVoltageCompensation(Constants.voltageComp);
    leftShooterMotor.enableVoltageCompensation(Constants.voltageComp);

    rightShooterMotor.setInverted(ShooterConstants.invertRightShooter);
    leftShooterMotor.setInverted(ShooterConstants.invertLeftShooter);

    rightShooterMotor.setIdleMode(ShooterConstants.rightShooterIdleMode);
    leftShooterMotor.setIdleMode(ShooterConstants.leftShooterIdleMode);

    leftShooterMotor.follow(rightShooterMotor);
  }

  /**
   * Configures the aux right and aux left shooter motors by resetting them and 
   * setting all necessary settings declared in Constants
   */
  private void configAuxShooterMotors(){
    auxRightShooterMotor.restoreFactoryDefaults();
    auxLeftShooterMotor.restoreFactoryDefaults();

    auxRightShooterMotor.enableVoltageCompensation(Constants.voltageComp);
    auxLeftShooterMotor.enableVoltageCompensation(Constants.voltageComp);

    auxRightShooterMotor.setInverted(ShooterConstants.invertAuxRightShooter);
    auxLeftShooterMotor.setInverted(ShooterConstants.invertAuxLeftShooter);

    auxRightShooterMotor.setIdleMode(ShooterConstants.auxRightShooterIdleMode);
    auxLeftShooterMotor.setIdleMode(ShooterConstants.auxLeftShooterIdleMode);

    auxLeftShooterMotor.follow(auxRightShooterMotor);
  }
```

In addition to the normal motor configurations like IdleMode, motor inverted, and restoring factory defaults, shooter subsystems generally have one more part when initializing motors.

The method enableVoltageCompensation is used to provide and compensate the voltage drawn by the motor. This allows for the motors to draw more when the battery is low in order for the motors to reach their desired speeds. 

Voltage compensation should only be needed for systems that need to function at their desired speeds. For example, this usually includes the swerve drive motors and shooter motors as these two systems are usually necessary to operate at their desired speeds. 

```
  public void shoot(){
    rightShooterMotor.set(ShooterConstants.shootSpeed);
    auxRightShooterMotor.set(ShooterConstants.auxShootSpeed);
  }
```

The shoot method sets the right motor of each motor pair to the set speed as defined in constants. 

Because the left motors were set to follow the right motors, only the right motors need set

DISCLAIMER: 
If using motor followers, if the lead motor suddenly fails during a match, then any follower motors will also fail when the lead motor fails. In order to prevent this, don't use motor followers and instead set each motor speed individually.

```
   /**
   * Steps each motor and sets their output to 0
   */
  public void stop(){
    rightShooterMotor.set(0);
    leftShooterMotor.set(0);
    auxRightShooterMotor.set(0);
    auxLeftShooterMotor.set(0);
  }
```

Once again, each subsystem will generally have a stop command as best practice. The shooter subsystem will have the stop command as the default command, and then when controlled, the shooter subsystem will respond and shoot the intended object.

```
public double getRPM(){
    return shooterEncoder.getVelocity();
}
```

Finally, if the shooter needs to rev up or reach a desired speed before shooting. The getRPM method will be useful to help check the RPM of the shooter motors. 

In this example, because the motors are directly connected to the axle of the shooter, there is no conversion factor needed to be set on the encoder. The getVelocity() method by default returns the RPM (rotations per minute) of the motor.

<div id="feed"></div>

# Feeders



<div id="intakeshoot"></div>

# Combining the Intake and Shooter

<div id="button"></div>

# Initializing Buttons and Commands