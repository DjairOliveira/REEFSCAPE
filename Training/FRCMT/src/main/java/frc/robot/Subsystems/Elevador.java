package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Elevador extends Command{
    private double Position=0;
    private double setP=0;
    private double OutRange=0;

    public Elevador(double  Position, double setP, double OutRange){
        this.Position = Position;
        this.setP = setP;
        this.OutRange = OutRange;

        // 86 Position max
    }

    //@Override
    public void initialize(){
        Constants.Elevador.confPrincipal.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Elevador.confPrincipal.openLoopRampRate(2);
        Constants.Elevador.confPrincipal.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Elevador.confPrincipal.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(setP, 0, 0)
        .outputRange(-OutRange, OutRange);

        Constants.Elevador.mPrincipal.configure(Constants.Elevador.confPrincipal, null, null);
        Constants.Elevador.pidController = Constants.Elevador.mPrincipal.getClosedLoopController();
    }

    public void execute(){
        Constants.Elevador.pidController.setReference(Position, ControlType.kPosition);
    }

    //@Override
    public boolean isFinished(){
        return true;
    }
}