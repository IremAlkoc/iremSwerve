package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Xbox denetleyici sınıfını ekliyoruz
import frc.robot.commands.Drive; // Drive komutunu ekliyoruz
import frc.robot.subsystems.DriveTrain; // DriveTrain (tahrik sistemi) sınıfını ekliyoruz

// RobotContainer sınıfı, robotun ana yapılandırmasını ve komutlarını içerir
public class RobotContainer {
    // Xbox denetleyici (sürücü) tanımlanıyor
    private final CommandXboxController m_driver;

    // DriveTrain (tahrik sistemi) tanımlanıyor
    final DriveTrain m_Drivetrain;

    // Drive komutunu tanımlıyoruz
    private final Drive m_Drive;

    // Yapıcı metot, robotun donanımını ve komutlarını yapılandırır
    public RobotContainer() {
        // Denetleyiciyi, Constants sınıfındaki port numarasına göre başlatıyoruz
        this.m_driver = new CommandXboxController(Constants.Controllers.Driver.kPort);

        // DriveTrain (tahrik sistemi) nesnesini oluşturuyoruz
        this.m_Drivetrain = new DriveTrain();

        // Drive komutunu, denetleyicinin joystick verilerini alacak şekilde yapılandırıyoruz
        this.m_Drive = new Drive(this.m_Drivetrain,
                () -> this.m_driver.getLeftX(),  // Joystick'in sol X ekseni (yön)
                () -> this.m_driver.getLeftY(),  // Joystick'in sol Y ekseni (yön)
                () -> this.m_driver.getRightX()); // Joystick'in sağ X ekseni (dönme)

        // Tahrik sisteminin varsayılan komutu, Drive komutu olarak ayarlanıyor
        this.m_Drivetrain.setDefaultCommand(m_Drive);

        // Buton bağlamalarını yapılandırıyoruz (şu an için boş)
        configureButtonBindings();
    }

    // Buton bağlamalarını burada yapılandırırız, ama şu an bir işlem yok
    private void configureButtonBindings() {
    }
}
