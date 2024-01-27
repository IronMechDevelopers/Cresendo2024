package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax shooterMotor;
    private CANSparkMax collectorMotor;
    private CANSparkMax stagingMotor;

    public ShooterSubsystem() {
        this.shooterMotor = new CANSparkMax(MotorIds.kShooterCanId, MotorType.kBrushless);
        this.collectorMotor = new CANSparkMax(MotorIds.kCollectorCanId, MotorType.kBrushless);
        this.stagingMotor = new CANSparkMax(MotorIds.kStagingCanId, MotorType.kBrushless);
    }

    /**
     * Control shooters power and speed.
     * 
     * @param shooterSpeed a number between 0 and 1 where 0 is off and 1 is 100%
     *                     power.
     */
    public void shooterPower(double shooterSpeed) {
        shooterMotor.set(shooterSpeed);
    }

    /**
     * Control collectors power and speed.
     * 
     * @param collectorSpeed a number between 0 and 1 where 0 is off and 1 is 100%
     *                       power.
     */
    public void collectorPower(double collectorSpeed) {
        collectorMotor.set(collectorSpeed);
    }

    /**
     * Control staging power and speed.
     * 
     * @param stagingSpeed a number between 0 and 1 where 0 is off and 1 is 100%
     *                     power.
     */
    public void stagingPower(double stagingSpeed) {
        stagingMotor.set(stagingSpeed);
    }

    /**
     * Turns Shooter, Collector and Staging speed to 0.
     */
    public void motorPowerOff() {
        shooterMotor.set(0);
        collectorMotor.set(0);
        stagingMotor.set(0);
    }
}
 