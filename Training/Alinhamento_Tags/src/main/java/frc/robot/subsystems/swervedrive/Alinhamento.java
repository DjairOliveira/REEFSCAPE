package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Limelight;

public class Alinhamento extends Command {
    private final SwerveSubsystem driveSubsystem;
    private final Limelight LimelightFront;
    private final BooleanSupplier isAligning;

    public Alinhamento(SwerveSubsystem drive, Limelight vision, BooleanSupplier isAligning) {
        this.driveSubsystem = drive;
        this.LimelightFront = vision;
        this.isAligning = isAligning; // Armazene o BooleanSupplier
        addRequirements(driveSubsystem); // Garante que outros comandos não usem o subsistema
    }
    private double calculateAlignmentSpeedX(double tx) {
        double kP = 0.1; // Constante proporcional
        return -kP * tx; // Negativo para alinhar corretamente
    }
    
    private double calculateAlignmentSpeedY(double ty) {
        double kP = 0.1;
        return -kP * ty;
    }
    
    private double calculateAlignmentRotation(double ta) {
        double kP = 0.1;
        return -kP * ta;
    }
    // Command alinhamentotuiCommand = driveSubsystem.driveCommand(
    //     () -> 0.3, // Movimentação para frente
    //     () -> 0.0, // Sem movimento lateral
    //     () -> 0.0  // Sem rotação
    //     );

    @Override
    public void execute() {
        double tx = LimelightFront.getTx();
        double ty = LimelightFront.getTy();
        double ta = LimelightFront.getTa();

        double xSpeed = isAligning.getAsBoolean() ? 0.2 : MathUtil.applyDeadband(RobotContainer.driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND);
        double ySpeed = isAligning.getAsBoolean() ? 0 : MathUtil.applyDeadband(RobotContainer.driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND);
        double rotationSpeed = isAligning.getAsBoolean() ? 0.3 : RobotContainer.driverXbox.getRightX() * -1;

        DoubleSupplier xSpeedSupplier = () -> xSpeed;
        DoubleSupplier ySpeedSupplier = () -> ySpeed;
        DoubleSupplier rotationSpeedSupplier = () -> rotationSpeed;

        // RobotContainer.driverXbox.rightBumper().whileTrue(Commands.run(() -> {
        // if (driveSubsystem.getDefaultCommand() != null) {
        //     driveSubsystem.getDefaultCommand().cancel(); // Interrompe o comando padrão atual
        // } 
        // driveSubsystem.setDefaultCommand(alinhamentotuiCommand); // Define o novo comando padrão
        // driveSubsystem.getDefaultCommand().schedule(); // Força a execução do novo comando
        // }));
        driveSubsystem.driveCommand(xSpeedSupplier, ySpeedSupplier, rotationSpeedSupplier);
        SmartDashboard.putNumber("XSpeed", xSpeed);
    }

    @Override
    public boolean isFinished() {
        // Condição para finalizar o alinhamento
        return Math.abs(LimelightFront.getTx()) < 0.1; // Exemplo: erro menor que 10cm
    }
}
