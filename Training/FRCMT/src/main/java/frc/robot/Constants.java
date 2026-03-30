package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Constants {

    public final class Elevador{
        public static final SparkMax mPrincipal = new SparkMax(21, MotorType.kBrushless);
        public static final SparkMaxConfig  confPrincipal = new SparkMaxConfig();
        public static SparkClosedLoopController pidController;
    }
}
