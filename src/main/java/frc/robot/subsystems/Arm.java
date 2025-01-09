package frc.robot.subsystems;

//These imports are for the motor controller used (CANSparkMaxes)
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

//Useful function interface that allows doubles to be supplied to a command.
import java.util.function.DoubleSupplier;

//Encoder is so we can measure rotation, which is useful for PID control
import com.revrobotics.RelativeEncoder;

//This is for our arm's precise control + general utility functions
//I used clamp in order to limit the values. It isn't hard to implement, but importing saves time
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//These make the command based programming magic happen
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//contains PID setpoints, PID & feedforward values, and other useful variables
//updating Constants.java will update all usage, so its best to keep them in a shared space
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    // instantiate motor controllers
    private SparkMax m_Arm;

    // instantiate encoders of the motor controllers
    public RelativeEncoder armEncoder;

    // create open loop control
    private ArmFeedforward armFeedforward = new ArmFeedforward(
            Constants.Arm.Feedforward.kS, Constants.Arm.Feedforward.kG, Constants.Arm.Feedforward.kV);

    // create closed loop control with restrictions
    private ProfiledPIDController armPidController = new ProfiledPIDController(
            Constants.Arm.PID.kP, Constants.Arm.PID.kI, Constants.Arm.PID.kD, new TrapezoidProfile.Constraints(
                    Constants.Arm.PID.constraints.kMAXV, Constants.Arm.PID.constraints.kMAXACC));

    private double armVoltage;// stores sum of pid and feedforward, and is applied to arm motor

    /*This is to manage whether or not the PID is enabled.
     * If false, nothing will really occur, as it is solely used to make a trigger.
     * Ignore if PID will be on at all times.
     */
    private boolean pidOn = true;

    public Arm() {
        //Create new brushless motor
        m_Arm = new SparkMax(Constants.Arm.CANIDs.ARM, SparkLowLevel.MotorType.kBrushless);

        //apply encoder conversions and smart current limits to arm motor
        SparkMaxConfig aConfig = new SparkMaxConfig();
        aConfig.smartCurrentLimit(Constants.Arm.ARM_CURRENT_LIMIT);
        aConfig.encoder.positionConversionFactor(Constants.Arm.ENCODER_TO_RADIANS);
        //the line that actually applies the changes made in the aConfig object
        m_Arm.configure(aConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //needed to measure where the arm is (for pid)
        armEncoder = m_Arm.getEncoder();

        //relatively accurate even with this tolerance. (Measurement in terms of radians)
        armPidController.setTolerance(0.15);

        //set a goal before anything starts so if none is set by default, robot won't act odd
        setArmGoal(Constants.Arm.PID.setpoints.GROUND);

    }

    /**
     * Set arm voltage to method input
     * 
     * @param voltage Voltage to set arm motor to (-12 to 12)
     */
    public void setArmVoltage(double voltage) {
        m_Arm.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    /**
     * Returns a command to control the arm manually using a lambda that returns the voltage
     * @param voltageSupplier Pass in a method that returns a double for your voltage
     * @return Command to run, which will set the voltage to the result of the method supplied
     */
    public Command armManualControl(DoubleSupplier voltageSupplier){
        return this.runOnce(()->setArmVoltage(voltageSupplier.getAsDouble())).withName("arm.manual");
    }

    /**
     * Sets the pid controller's goal to the input provided
     * @param goal {@code double} value to set the arm's goal to
     */
    public void setArmGoal(double goal) {
        armPidController.reset(armEncoder.getPosition());
        armPidController.setGoal(goal);
    }

    /**
     * Command factory that will allow arbitrary goals to be set for the arm {@link #armPidController PID}
     * @param goal {@code double} value that the goal will be set to (in radians)
     * @return Command that sets the arm goal to provided input
     */
    public Command setArmGoalCommand(double goal){
        return this.runOnce(()->setArmGoal(goal)).withName("arm.setGoal");
    }

    /**
     * <pre>
     *Rotates the arm to the goal set by {@link #setArmGoal(double)}
     *Output is clamped to safe voltages
     * </pre>
     */
    public void rotateArmToGoal() {
            double feedforward = armFeedforward.calculate(armPidController.getSetpoint().position,
                    armPidController.getSetpoint().velocity);
            double pid = armPidController.calculate(armEncoder.getPosition());
            armVoltage = feedforward + pid;
        setArmVoltage(armVoltage);//built in clamp to ensure safe values
    }

    /**
     * Creates a command that will rotate the arm to the set goal, using {@link #armPidController PID}
     *  and {@link #armFeedforward feedforward}
     * @return Command that rotates the arm to the goal
     */
    public Command rotateArmToGoalCommand(){
        return Commands.sequence(
            resetPID(),
            this.run(()->{
                rotateArmToGoal();
            })
        ).withName("arm.rotateArmToGoal");
    }

    /**
     * Command factory that builds upon {@link #rotateArmToGoalCommand()}, allowing a goal to be set
     * @param goal The goal to rotate the arm to (in radians)
     * @return A command that will rotate the arm until it reaches the goal provided
     */
    public Command rotateArmToNewGoalCommand(double goal){
        return Commands.sequence(
            resetPID(),
            setArmGoalCommand(goal),
            rotateArmToGoalCommand().until(this::armIsAtTarget)
        ).withName("arm.rotateToNewGoal");
    }

    /**
     * Stops the arm. This will cause the motor to brake.
     * @return Command that stops the {@link #m_Arm arm motor}
     */
    public Command stopArm() {
        return this.runOnce(()->m_Arm.stopMotor()).withName("arm.stop");
    }

    /**
     * Resets the arm and intake encoders so the current position is 0
     * @return Command which resets the {@link #armEncoder arm's encoder}
     */
    public Command resetEncoders() {
        return this.runOnce(()->{
            armEncoder.setPosition(0);
        }).withName("arm.resetEncoders");
    }

    /**
     * Checks if the arm is at the target set by {@link #setArmGoal(double)}
     * or commands that set the arm goal
     * @return True if at the goal, False otherwise
     */
    public boolean armIsAtTarget() {
        return armPidController.atGoal();
    }

    /**
     * Resets the PID controller for the arm to the current encoder position
     * @return Command which resets the {@link #armPidController arm's PID controller}
     */
    public Command resetPID() {
        return this.runOnce(()->armPidController.reset(getPosition())).withName("arm.resetPID");
    }

    /**
     * Method which finds the position of the arm using {@link #armEncoder the arm's encoder}
     * @return Position of the arm, as type double, in radians. (0 is the highest point)
     */
    public double getPosition() {
        return armEncoder.getPosition();
    }

    /**
     * Simple getter for the {@link #pidOn} variable
     * @return Value of {@link #pidOn}
     */
    public boolean getPidOn(){
        return pidOn;
    }

    /**
     * Toggles between true and false for {@link #pidOn}.
     * If pid is being enabled, resets the pid controller to prevent issues
     * @return Command to toggle {@link #pidOn} and reset {@link #armPidController the arm's pid controller}
     */
    public Command togglePid(){
        return this.runOnce(()->{
            pidOn = !pidOn;
            if (pidOn){
                armPidController.reset(getPosition());
            }
        }).withName("arm.togglePIDEnabled");
    }

}
