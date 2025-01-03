package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {  // SwerveModule sınıfı, swerve tahrik modülünü temsil eder.
    private int m_moduleNumber;  // Modül numarası, bu modülün robot üzerindeki hangi modül olduğunu belirtir.

    private final CANSparkMax m_driveMotor;  // Tahrik motoru (sürüş motoru) CANSparkMax motoru.
    private final CANSparkMax m_angleMotor;  // Açılama motoru (swerve tahrik yönü motoru).

    private final boolean m_driveMotorInvert;  // Sürüş motorunun ters çalışıp çalışmayacağını belirler.
    private final boolean m_angleMotorInvert;  // Açılama motorunun ters çalışıp çalışmayacağını belirler.

    private final RelativeEncoder m_driveRelativeEncoder;  // Sürüş motoru için göreli enkoder.
    private final RelativeEncoder m_angleRelativeEncoder;  // Açılama motoru için göreli enkoder.

    private final CANcoder m_angleAbsoluteEncoder;  // Açılama motorunun mutlak enkoderi (yön ölçümü).
    private final double m_angleAbsoluteEncoderOffset;  // Mutlak enkoderdeki ofset değeri (kalibrasyon).

    private SparkPIDController m_driveMotorSparkPIDController;  // Tahrik motorunun PID kontrolörü.
    private PIDController m_angleMotorPIDController;  // Açılama motorunun PID kontrolörü.

    private SwerveModuleState m_state;  // Modülün mevcut durumu (hız ve yön).
    private Pose2d m_pose;  // Modülün pozisyonu.

    // Swerve modülü için yapılandırıcı (constructor), motorlar ve enkoderler için ID'ler alır.
    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, boolean driveMotorInvert,
            boolean angleMotorInvert, int angleAbsoluteEncoderID,
            double angleAbsoluteEncoderOffset) {

        this.m_moduleNumber = moduleNumber;  // Modül numarasını atar.
        this.m_driveMotorInvert = driveMotorInvert;  // Sürüş motorunun ters çalışıp çalışmayacağını ayarlar.
        this.m_angleMotorInvert = angleMotorInvert;  // Açılama motorunun ters çalışıp çalışmayacağını ayarlar.
        this.m_driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);  // Tahrik motorunu oluşturur.
        this.m_driveMotor.restoreFactoryDefaults();  // Fabrika ayarlarına sıfırlar.
        this.m_driveMotor.setSmartCurrentLimit(45);  // Sürüş motoru için akım sınırını belirler.
        this.m_driveMotor.getPIDController().setFF(0.0);  // PID kontrolörüne doğrusal özellik ekler.
        this.m_driveMotor.getPIDController().setP(0.2);  // PID kontrolörüne Proportional (P) katsayısını ayarlar.
        this.m_driveMotor.getPIDController().setI(0.0);  // PID kontrolörüne Integral (I) katsayısını ayarlar.
        this.m_driveMotor.setInverted(this.m_driveMotorInvert);  // Motorun ters olup olmadığını belirler.
        this.m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);  // Çerçeve güncelleme süresi.
        this.m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);  // Çerçeve güncelleme süresi.
        this.m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);  // Çerçeve güncelleme süresi.
        this.m_driveMotor.enableVoltageCompensation(12.6);  // Voltaj dengelemesini etkinleştirir.
        this.m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);  // Motorun boşta çalışma modunu frenleme olarak ayarlar.

        this.m_angleMotor = new CANSparkMax(angleMotorID, CANSparkLowLevel.MotorType.kBrushless);  // Açılama motorunu oluşturur.
        this.m_angleMotor.restoreFactoryDefaults();  // Fabrika ayarlarına sıfırlar.
        this.m_angleMotor.setSmartCurrentLimit(20);  // Açılama motoru için akım sınırını belirler.
        this.m_angleMotor.getPIDController().setFF(0.0);  // PID kontrolörüne doğrusal özellik ekler.
        this.m_angleMotor.getPIDController().setP(0.2);  // PID kontrolörüne Proportional (P) katsayısını ayarlar.
        this.m_angleMotor.getPIDController().setI(0.0);  // PID kontrolörüne Integral (I) katsayısını ayarlar.
        this.m_angleMotor.setInverted(this.m_angleMotorInvert);  // Motorun ters olup olmadığını belirler.
        this.m_angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);  // Çerçeve güncelleme süresi.
        this.m_angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);  // Çerçeve güncelleme süresi.
        this.m_angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);  // Çerçeve güncelleme süresi.
        this.m_angleMotor.enableVoltageCompensation(12.6);  // Voltaj dengelemesini etkinleştirir.
        this.m_angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);  // Motorun boşta çalışma modunu frenleme olarak ayarlar.

        this.m_driveRelativeEncoder = this.m_driveMotor.getEncoder();  // Tahrik motorunun göreli enkoderini alır.
        this.m_driveRelativeEncoder.setPositionConversionFactor(Constants.Swerve.Drive.kRevolutionsToMeters);  // Pozisyon dönüşüm faktörü belirler.
        this.m_driveRelativeEncoder.setVelocityConversionFactor(Constants.Swerve.Drive.kRPMtoMPS);  // Hız dönüşüm faktörü belirler.

        this.m_angleRelativeEncoder = this.m_angleMotor.getEncoder();  // Açılama motorunun göreli enkoderini alır.
        this.m_angleRelativeEncoder.setPositionConversionFactor(Constants.Swerve.Angle.kRevolutionsToDegrees);  // Açıyı dereceye dönüştürür.
        this.m_angleRelativeEncoder.setVelocityConversionFactor(Constants.Swerve.Angle.kRevolutionsToDegrees / 60.0);  // Açıyı hıza dönüştürür.

        this.m_angleAbsoluteEncoder = new CANcoder(angleAbsoluteEncoderID);  // Açılama motorunun mutlak enkoderini oluşturur.
        this.m_angleAbsoluteEncoderOffset = angleAbsoluteEncoderOffset;  // Mutlak enkoderdeki ofset değerini ayarlar.

        this.m_driveMotorSparkPIDController = this.m_driveMotor.getPIDController();  // Sürüş motoru için PID denetleyicisini alır.
        this.m_angleMotorPIDController = new PIDController(0.007, 0.00175, 0.0000625);  // Açılama motoru için PID denetleyicisini oluşturur.
    }

        // Bu fonksiyon, açı enkoderinin göreli pozisyonunu mutlak enkoderin değerine göre sıfırlar.
    public void resetAngleRelativeEncoderToAbsolute() {
        double angle = (this.m_angleAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0)
                - this.m_angleAbsoluteEncoderOffset;  // Mutlak enkoderin değerini dereceye dönüştürür ve ofseti ekler.
        this.m_angleRelativeEncoder.setPosition(angle);  // Göreli enkoderin pozisyonunu günceller.
    }

    // Bu fonksiyon, sürüş motorunun göreli enkoderinin pozisyonunu sıfırlar.
    public void resetDriveRelativeEncoder() {
        this.m_driveRelativeEncoder.setPosition(0);  // Göreli enkoderin pozisyonunu sıfırlar.
    }

    /**
     * Bu fonksiyon, modül numarasını döndürür.
     * @return modül numarasını döndürür.
     */
    public int getModuleNumber() {
        return this.m_moduleNumber;  // Modül numarasını döndürür.
    }

    /**
     * Bu fonksiyon, modülün yönünü derece cinsinden döndürür.
     * @return modülün yönü (derece cinsinden).
     */
    public double getHeadingDegrees() {
        return this.m_angleRelativeEncoder.getPosition();  // Açılama motorunun göreli enkoderinin pozisyonunu döndürür.
    }

    /**
     * Bu fonksiyon, modülün yönünü Rotation2d sınıfı ile döndürür.
     * @return modülün yönü (Rotation2d nesnesi).
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());  // Dereceyi Rotation2d nesnesine dönüştürür.
    }

    /**
     * Bu fonksiyon, modülün toplam ne kadar mesafe katettiğini metre cinsinden döndürür.
     * @return toplam mesafe (metre cinsinden).
     */
    public double getDriveMeters() {
        return this.m_driveRelativeEncoder.getPosition();  // Sürüş motorunun göreli enkoderinin pozisyonunu döndürür.
    }

    /**
     * Bu fonksiyon, modülün mevcut hızını metre/saniye cinsinden döndürür.
     * @return modülün hızı (metre/saniye cinsinden).
     */
    public double getDriveMetersPerSecond() {
        return this.m_driveRelativeEncoder.getVelocity();  // Sürüş motorunun göreli enkoderinin hızını döndürür.
    }

    /**
     * Bu fonksiyon, modülün mevcut durumunu döndürür.
     * @return mevcut modül durumu (SwerveModuleState).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());  // Modülün hızı ve yönünü döndürür.
    }

    /**
     * Bu fonksiyon, modülün konumunu döndürür. Konum, toplam kat edilen mesafe ve mevcut yönü içerir.
     * @return modülün konumu (SwerveModulePosition).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());  // Modülün konumunu döndürür.
    }

    /**
     * Bu fonksiyon, modülün mevcut pozisyonunu (Pose2d) döndürür.
     * @return modülün pozisyonu (Pose2d).
     */
    public Pose2d getModulePose() {
        return this.m_pose;  // Modülün pozisyonunu döndürür.
    }

    /**
     * Bu fonksiyon, modülün pozisyonunu (Pose2d) ayarlar.
     * @param pose yeni pozisyon (Pose2d).
     */
    public void setModulePose(Pose2d pose) {
        this.m_pose = pose;  // Modülün pozisyonunu ayarlar.
    }

    /**
     * Bu fonksiyon, modülü belirli bir hedef duruma (SwerveModuleState) göre ayarlar.
     * Modülün yönü ve hızı aynı anda kontrol edilir.
     * @param desiredState hedef durum (SwerveModuleState).
     * @param isOpenLoop sürüş motorunun kontrol tipi (açık döngü mü, kapalı döngü mü).
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        this.m_state = optimize(desiredState, getHeadingRotation2d());  // Hedef durumu optimize eder.

        if (isOpenLoop) {
            // Açık döngüde sürüş motorunun güç yüzdesini hesaplar.
            double percentOutput = this.m_state.speedMetersPerSecond / Constants.Swerve.MaxSpeeds.kTranslation;
            this.m_driveMotor.set(percentOutput);  // Motoru hedef hızla çalıştırır.
        } else {
            // Kapalı döngüde sürüş motorunun PID kontrolörünü hedef hızla çalıştırır.
            this.m_driveMotorSparkPIDController.setReference(
                    this.m_state.speedMetersPerSecond,
                    CANSparkMax.ControlType.kVelocity,
                    1);  // Hedef hıza ulaşmak için PID kontrolü kullanır.
        }

        // Dönme motorunu hedef açıya yönlendirir.
        turnToDegrees(this.m_state.angle.getDegrees());
    }

    /**
     * Bu fonksiyon, modülü belirli bir hedef açıya yönlendirir.
     * @param angle hedef açı (derece cinsinden).
     */
    public void turnToDegrees(double angle) {
        double turnAngleError = Math.abs(angle - this.m_angleRelativeEncoder.getPosition());  // Hedef açı ile mevcut açı arasındaki farkı hesaplar.

        double pidOut = this.m_angleMotorPIDController.calculate(this.m_angleRelativeEncoder.getPosition(), angle);  // PID kontrolü ile dönme motorunun çıkışını hesaplar.
        
        // Eğer robot hareket etmiyorsa, dönme motorunun salınımını durdurur.
        if (turnAngleError < .5
                && Math.abs(this.m_state.speedMetersPerSecond) <= Constants.Swerve.MaxSpeeds.kTranslation * .01)
                
            pidOut = 0;  // Eğer robot çok yavaşsa, dönme motoru durdurulur.

        this.m_angleMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());  // Motor voltajını ayarlar.
    }

    /**
     * Bu fonksiyon, hedef durumu optimize eder. Hedef açı ve hız, robotun mevcut yönüne göre ayarlanır.
     * @param desiredState hedef durum (SwerveModuleState).
     * @param currentAngle mevcut yön (Rotation2d).
     * @return optimize edilmiş hedef durum (SwerveModuleState).
     */
    public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());  // Hedef açıyı mevcut açıya göre ayarlar.
        double targetSpeed = desiredState.speedMetersPerSecond;  // Hedef hızı alır.
        double delta = targetAngle - currentAngle.getDegrees();  // Hedef açı ile mevcut açı arasındaki fark.

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;  // Hedef açı 90 dereceyi geçerse, hızın yönünü tersine çevirir.
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);  // Açıyı ters yöne döndürür.
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));  // Optimize edilmiş durumu döndürür.
    }

    /**
     * Bu fonksiyon, hedef açıyı 0 ile 360 derece arasında doğru bir şekilde yerleştirir.
     * @param scopeReference mevcut açı (mevcut yön).
     * @param newAngle hedef açı (derece cinsinden).
     * @return hedef açı (0 ile 360 derece arasında).
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;

        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

        while (newAngle < lowerBound)  // Hedef açı, alt sınırdan küçükse 360 ekler.
            newAngle += 360;
        while (newAngle > upperBound)  // Hedef açı, üst sınırdan büyükse 360 çıkarır.
            newAngle -= 360;

        if (newAngle - scopeReference > 180)  // Eğer açı 180 dereceyi geçerse, 360 çıkarır.
            newAngle -= 360;
        else if (newAngle - scopeReference < -180)  // Eğer açı -180 dereceden küçükse, 360 ekler.
            newAngle += 360;

        return newAngle;  // Hedef açıyı döndürür.
    }


    
       // Bu fonksiyon, her 20 milisaniyede bir çağrılır. Modülün durumunu ekrana yazdırır.
       @Override
       public void periodic() {
           SmartDashboard.putNumber("Module " + this.m_moduleNumber + " Position", getHeadingDegrees());  // Modülün pozisyonunu ekrana yazdırır.
           SmartDashboard.putNumber("Module " + this.m_moduleNumber + " Speed", getDriveMetersPerSecond());  // Modülün hızını ekrana yazdırır.
       }  
   }


