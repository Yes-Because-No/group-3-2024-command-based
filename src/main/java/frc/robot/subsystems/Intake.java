package frc.robot.subsystems;

//not really used but I suppose it is fine to import in case you do use PID
import com.revrobotics.RelativeEncoder;

//as in all the other subsystems, CANSparkMax imports
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

//command based imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//constants for the intake
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    //instantiate intake motor & corresponding encoder
    private SparkMax m_Intake;
    private RelativeEncoder intakeEncoder;

    public Intake(){
        //give the motor an actual value
        m_Intake = new SparkMax(Constants.Arm.CANIDs.INTAKE, SparkLowLevel.MotorType.kBrushless);

        //now create and apply the configurations
        SparkMaxConfig iConfig = new SparkMaxConfig();
        iConfig.smartCurrentLimit(Constants.Arm.INTAKE_CURRENT_LIMIT);
        m_Intake.configure(iConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        //finally, get your (basically unused) encoder
        intakeEncoder = m_Intake.getEncoder();

    }
    /**
     * Spins the {@link #m_Intake intake} motor in order to take in game pieces
     * @return Command that will start the intake motor's spinning to intake
     */
    public Command intake() {
        return this.runOnce(()->{m_Intake.setVoltage(3.5);});// change later
    }

    /**
     * Spins the {@link #m_Intake intake} motor in order to push out game pieces
     * @return Command that will start the intake motor's spinning to outtake
     */
    public Command outtake() {
        return this.runOnce(()->m_Intake.setVoltage(-6));// change later
    }

    /**
     * Stops taking in/out game pieces by stopping the {@link #m_Intake motor}.
     * This should have the motor brake, as the Spark Max
     * has been configured to do so.
     * @return Command to stop intake motor motion
     */
    public Command stopIntake() {
        return this.runOnce(()->m_Intake.stopMotor());
    }

    /**
     * Reset the {@link #m_Intake intake} motor's {@link #intakeEncoder encoder}
     * @return Command that resets intake encoder
     */
    public Command resetEncoders(){
        return this.runOnce(()->intakeEncoder.setPosition(0));
    }



}
