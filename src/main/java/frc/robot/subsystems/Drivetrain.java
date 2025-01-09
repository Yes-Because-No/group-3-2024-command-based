package frc.robot.subsystems;

//used to make the CANSparkMax motor controllers work with custom configuration
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

//used to allow the passing of lambdas to commands (for arcade/tankdrive control)
import java.util.function.DoubleSupplier;

//get motor encoder values. Not really used as we don't have PID for this subsystem
import com.revrobotics.RelativeEncoder;

//soften inputs to controls
import edu.wpi.first.math.filter.SlewRateLimiter;

//the wpilib class that provides easy access to tank/arcade drive & more
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//needed for command based
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    //create motors and encoders (null values)
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    //make your drivetrain object
    private DifferentialDrive drivetrain;

    //make arcade drive limiter
    private SlewRateLimiter filterArcade;

    public Drivetrain() {
        // motors
        m_leftMotor = new SparkMax(Constants.Drivetrain.CANIDs.LEFT, SparkLowLevel.MotorType.kBrushless);
        m_rightMotor = new SparkMax(Constants.Drivetrain.CANIDs.RIGHT, SparkLowLevel.MotorType.kBrushless);

        // provides arcade drive
        drivetrain = new DifferentialDrive(m_leftMotor, m_rightMotor);

        // softens arcade drive motion
        filterArcade = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT);

        //setup left motor settings
        SparkMaxConfig configLeft = new SparkMaxConfig();
        configLeft
            .inverted(false)
            .smartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
        
        //and then right motor settings
        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight
            .inverted(true)
            .smartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);

        //now actually apply the motor settigs
        m_leftMotor.configure(configLeft,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_rightMotor.configure(configRight,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
        // to check distances traveled if we end up using this (hint: we didn't)
        leftEncoder = m_leftMotor.getEncoder();
        rightEncoder = m_rightMotor.getEncoder();
    }

    /**
     * Implements arcade drive with softened forward/back inputs for smoother control
     * @param speed Forwards/backwards motion (-1.0 to 1.0)
     * @param rotation Whether or not you rotate
     */
    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(filterArcade.calculate(speed), rotation);
    }

    /**
     * @param leftSpeed  The value you want your left speed to be (-1.0 to 1.0)
     * @param rightSpeed The value you want your right speed to be (-1.0 to 1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_leftMotor.setVoltage(leftSpeed * 12.0);
        m_rightMotor.setVoltage(rightSpeed * 12.0);
    }
    
    /**
     * Command that implements manual control of the {@link #drivetrain} using arcade drive
     * @param speed Lambda that will return the forwards/backwards speed (-1.0 to 1.0)
     * @param rotation Lambda that will return the rotational speed (-1.0 to 1.0)
     * @return Command that will continuously call the supplied lambdas and set the arcade drive 
     * values to the results.
     */
    public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation){
        return this.run(()->{
            arcadeDrive(speed.getAsDouble(), rotation.getAsDouble());
        });
    }

    /**
     * Command that implements manual control of the {@link #drivetrain} using tank drive
     * @param leftSpeed Lambda that will return the left speed (-1.0 to 1.0)
     * @param rightSpeed Lambda that will return the right speed (-1.0 to 1.0)
     * @return Command that will continuously call the supplied lambdas and set the tank drive
     * values to the results
     */
    public Command tankDriveCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){
        return this.run(()->{
            tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
        });
    }

    /**
     * Zero both {@link #drivetrain} motor encoders ({@link #rightEncoder right} and {@link #leftEncoder left}).
     */
    public void resetEncoders() {
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    /**
     * Can be used to temporarily bypass motor safety. This probably should never make it to 
     * an actual competition robot, and for group 3, was mainly for debug.
     */
    public void drivetrainFeed() {
        drivetrain.feed();
    }

}
