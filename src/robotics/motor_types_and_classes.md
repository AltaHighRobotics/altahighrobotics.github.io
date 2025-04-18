---
title: Motor types and classes
parent: Robotics 
---

___
# Motors
(Got bored while working on this, will finish later (not really))
## NEO brushless motor
![[Pasted image 20250313075024.png|100]]


## NEO 550 brushless (outside rotation)
![[Pasted image 20250313074727.png|100]]


## Talon 500 FX
![[Pasted image 20250313074756.png|100]]
##### Example code for subsystem:
```java
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

  

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PenetratorConstants;
  
public class PenetratorSubsystem extends SubsystemBase{
public enum MoveStyle {
INSERT,
PULLOUT,
STOP
}
  
private TalonFX motor;
  
	public PenetratorSubsystem() {
		super();
		this.motor = new TalonFX(PenetratorConstants.MOTOR_ID);
		  
		// final double extension = this.motor.getPosition().getValue().magnitude();
		// SmartDashboard.putNumber("Penetrator Extension", extension);
	}
  
	public void set(MoveStyle pegginType) {
		switch (pegginType) {
		case INSERT:
		this.motor.set(-PenetratorConstants.MOTOR_INSERT_SPEED);
		break;
		case PULLOUT:
		this.motor.set(PenetratorConstants.MOTOR_PULLOUT_SPEED);
		break;
		case STOP:
		this.motor.set(0);
		break;
		}
	}
}
```


## NEO Vortex
![[Pasted image 20250313074908.png|100]]


# Motor Controllers
## Spark max
![[Pasted image 20250313074830.png|100]]
Does not have a hard brake mode, can be configured using the REV configuration tool
### Example code:
#### Subsystem (for a brushless motor)
```java
package frc.robot.subsystems;
  
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SparkExampleConstants;
  
public class SparkExampleSubsystem extends SubsystemBase{
	// Make an enum that can be used to easily read/make commands to this subsystem
	public enum MoveStyle {
		FORWARD,
		BACKWARD,
		STOP
	}

	// Create SparkMax object and grab encoder
	private SparkMax motor;
	private RelativeEncoder turnEncoder;
	
	  
	public SparkExampleSubsystem() {
		super();
		// Initialize motor controller
		this.motor = new SparkMax(RimmerConstants.SPARK_MAX_ID, MotorType.kBrushless); // This part can be changed to use a brushed motor instead
		// Get encoder information
		final Rotation2d encoderRotation = this.getEncoder();
		SmartDashboard.putNumber("Rimmer Encoder Extension", encoderRotation.getRadians());
	}

	
	public void set(MoveStyle MoveType) {
		switch (MoveType) {
			case FORWARD:
			this.motor.set(-SparkExampleConstants.MOTOR_FORWARD_SPEED);
			break;
			case BACKWARD:
			this.motor.set(SparkExampleConstants.MOTOR_BACKWARD_SPEED);
			break;
			case STOP:
			this.motor.set(0);
			break;
		}
	}
}
```
#### Command (makes the robot move forward)
```java
package frc.robot.commands.lift; // Change this to be the name of your folder this command is stored in
  
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SparkExampleSubsystem;
import frc.robot.subsystems.SparkExampleSubsystem.MoveStyle;
  
/** An example command that uses an example subsystem. */
public class SparkMoveForward extends Command {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final SparkExampleSubsystem subsystem;
	  
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public SparkMoveForward(SparkExampleSubsystem subsystem) {
		this.subsystem = subsystem;
		addRequirements(subsystem);
	}
  
	/**
	* We need to use full power to get the ball in but no:t to keep it
	* So after a few seconds we lower the motor power
	*/
	@Override
	public void execute() {
		this.subsystem.set(MoveStyle.INSERT);
	}
	@Override
	public void end(boolean interrupted) {
		this.subsystem.set(MoveStyle.STOP);
	}
}
```