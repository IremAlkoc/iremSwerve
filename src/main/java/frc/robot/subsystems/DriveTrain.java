    package frc.robot.subsystems;

    import frc.robot.Constants;
    import frc.robot.Constants.Swerve.ModulePosition;

    import java.util.HashMap;
    import java.util.Map;

    import com.kauailabs.navx.frc.AHRS;

    import edu.wpi.first.math.MathUtil;
    import edu.wpi.first.math.filter.SlewRateLimiter;
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
    import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
    import edu.wpi.first.math.kinematics.SwerveModulePosition;
    import edu.wpi.first.math.kinematics.SwerveModuleState;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;

    public class DriveTrain extends SubsystemBase {
        // Robotun dört tekerleği için swerve modülleri
        private final SwerveModule m_FL; // Ön Sol Tekerlek
        private final SwerveModule m_FR; // Ön Sağ Tekerlek
        private final SwerveModule m_BL; // Arka Sol Tekerlek
        private final SwerveModule m_BR; // Arka Sağ Tekerlek

        // Gyro sensörü, robotun yönünü ölçmek için kullanılır
        private final AHRS m_gyro;

        // Modül konumlarını belirten bir harita
        private final HashMap<ModulePosition, SwerveModule> m_swerveModulesToPositions;

        // Robotun odometrisi (pozisyon takibi için)
        private SwerveDriveOdometry m_odometry;

        // Hız değişimlerini yumuşatmak için sınırlayıcılar
        private final SlewRateLimiter m_strafeSlewRateLimiter;
        private final SlewRateLimiter m_forwardSlewRateLimiter;
        private final SlewRateLimiter m_rotateSlewRateLimiter;

        // Alan merkezli veya robot merkezli hareket modu kontrolü
        private boolean isFieldCentric;

        public DriveTrain() {
            // Her bir tekerlek modülü için nesne oluşturma
            this.m_FL = new SwerveModule(0, Constants.Swerve.FL.kDriveMotorID, Constants.Swerve.FL.kAngleMotorID,
                    Constants.Swerve.FL.kDriveInvert, Constants.Swerve.FL.kAngleInvert,
                    Constants.Swerve.FL.kAngleAbsoluteEncoderID, Constants.Swerve.FL.kAngleAbsoluteEncoderOffset);
            this.m_FR = new SwerveModule(1, Constants.Swerve.FR.kDriveMotorID, Constants.Swerve.FR.kAngleMotorID,
                    Constants.Swerve.FR.kDriveInvert, Constants.Swerve.FR.kAngleInvert,
                    Constants.Swerve.FR.kAngleAbsoluteEncoderID, Constants.Swerve.FR.kAngleAbsoluteEncoderOffset);
            this.m_BL = new SwerveModule(2, Constants.Swerve.RL.kDriveMotorID, Constants.Swerve.RL.kAngleMotorID,
                    Constants.Swerve.RL.kDriveInvert, Constants.Swerve.RL.kAngleInvert,
                    Constants.Swerve.RL.kAngleAbsoluteEncoderID, Constants.Swerve.RL.kAngleAbsoluteEncoderOffset);
            this.m_BR = new SwerveModule(3, Constants.Swerve.RR.kDriveMotorID, Constants.Swerve.RR.kAngleMotorID,
                    Constants.Swerve.RR.kDriveInvert, Constants.Swerve.RR.kAngleInvert,
                    Constants.Swerve.RR.kAngleAbsoluteEncoderID, Constants.Swerve.RR.kAngleAbsoluteEncoderOffset);

            // Gyro sensörünü başlatma ve sıfırlama
            this.m_gyro = new AHRS();
            this.m_gyro.reset();

            // Tekerlek modüllerini pozisyonlarıyla eşleştiren bir harita oluşturma
            this.m_swerveModulesToPositions = new HashMap<>(
                    Map.of(
                            ModulePosition.FL, this.m_FL,
                            ModulePosition.FR, this.m_FR,
                            ModulePosition.RL, this.m_BL, 
                            ModulePosition.RR, this.m_BR 
                    ));

            // Odometriyi başlatma
            this.m_odometry = new SwerveDriveOdometry(Constants.Swerve.kSwerveDriveKinematics, getHeadingRotation2d(),
                    getModulePositions(), new Pose2d());

            // Hız sınırlayıcılarını başlatma
            this.m_strafeSlewRateLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.kStrafeSlewRateLimit);
            this.m_forwardSlewRateLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.kForwardsSlewRateLimit);
            this.m_rotateSlewRateLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.kRotateSlewRateLimit);
        }

        /**
         * Robotu joystick girdilerine göre sürmek için kullanılır.
         * 
         * @param inputX   Sol/sağ hareket girdisi
         * @param inputY   İleri/geri hareket girdisi
         * @param inputRot Dönme girdisi
         */
        public void drive(double inputX, double inputY, double inputRot) {
            // Ölü bölge uygulaması: Küçük girişleri sıfıra yuvarlama
            double deadbandedX = MathUtil.applyDeadband(Math.abs(inputX),
                    Constants.Controllers.Driver.kTranslationDeadband) * Math.signum(inputX);
            double deadbandedY = MathUtil.applyDeadband(Math.abs(inputY),
                    Constants.Controllers.Driver.kTranslationDeadband) * Math.signum(inputY);
            double deadbandedRot = MathUtil.applyDeadband(Math.abs(inputRot),
                    Constants.Controllers.Driver.kRotationDeadband) * Math.signum(inputRot);

            // Girişleri karesel olarak artırma, hassas kontrol için
            deadbandedX = -Math.signum(deadbandedX) * Math.pow(deadbandedX, 2);
            deadbandedY = -Math.signum(deadbandedY) * Math.pow(deadbandedY, 2);
            deadbandedRot = -Math.signum(deadbandedRot) * Math.pow(deadbandedRot, 2);

            // Hız sınırlayıcıları kullanarak girişleri yumuşatma
            double slewedX = this.m_strafeSlewRateLimiter.calculate(deadbandedX);
            double slewedY = this.m_forwardSlewRateLimiter.calculate(deadbandedY);
            double slewedRot = this.m_rotateSlewRateLimiter.calculate(deadbandedRot);

            // İşlenmiş girdileri motorlara gönderme
            sendDrive(slewedX, slewedY, slewedRot, true);
        }

        /**
         * Robotu kontrol etmek için hesaplanmış hareket komutlarını kullanır.
         * 
         * @param translationX Sol/sağ hareket girdisi (metre/saniye)
         * @param translationY İleri/geri hareket girdisi (metre/saniye)
         * @param rotation     Dönüş girdisi (radyan/saniye)
         * @param isOpenLoop   Hız kontrol modu (true: güç, false: hız)
         */
        public void sendDrive(double translationX, double translationY, double rotation, boolean isOpenLoop) {
            // Maksimum hız değerleriyle ölçeklendirme
            translationY *= Constants.Swerve.MaxSpeeds.kTranslation;
            translationX *= Constants.Swerve.MaxSpeeds.kTranslation;
            rotation *= Constants.Swerve.MaxSpeeds.kRotation;

            // Alan merkezli veya robot merkezli hesaplama
            ChassisSpeeds chassisSpeeds = isFieldCentric
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            translationY, translationX, rotation, getHeadingRotation2d())
                    : new ChassisSpeeds(translationY, translationX, rotation);

            // Şase hızlarını tekerlek hızlarına çevirme
            SwerveModuleState[] moduleStates = Constants.Swerve.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // Hızları normalize etme (maksimum hız aşıldığında)
            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.MaxSpeeds.kTranslation);

            // Her bir modüle hesaplanmış hızları gönderme
            for (SwerveModule module : this.m_swerveModulesToPositions.values())
                module.setDesiredState(moduleStates[module.getModuleNumber()], isOpenLoop);
        }

        /** @return Robotun şu anki yönünü (baş açısını) derece cinsinden döndürür */
        public double getHeadingDegrees() {
            // Gyro sensöründen alınan açıyı normalize ederek -360 ile 360 derece arasında
            // döndürür
            return -Math.IEEEremainder(this.m_gyro.getAngle(), 360);
        }

        /**
         * @return Robotun şu anki yönünü bir {@link Rotation2d} nesnesi olarak döndürür
         */
        public Rotation2d getHeadingRotation2d() {
            // Derece cinsinden baş açısını Rotation2d nesnesine çevirir
            return Rotation2d.fromDegrees(getHeadingDegrees());
        }

        /**
         * Robotun baş açısını sıfırlayarak, sahadaki yönünü etkili bir şekilde
         * değiştirir
         */
        public void resetHeading() {
            // Gyro sensörünü sıfırlar
            this.m_gyro.reset();
        }

        /**
         * @return Robotun metre cinsinden pozisyonunu ve derece cinsinden yönünü bir
         *         {@link Pose2d} nesnesi olarak döndürür
         */
        public Pose2d getPoseMeters() {
            // Odometri tarafından hesaplanan robotun pozisyonunu döndürür
            return this.m_odometry.getPoseMeters();
        }

        /**
         * @param moduleNumber Modülün indeksi
         * @return Verilen indeksteki {@link SwerveModule swerve modülü}
         */
        public SwerveModule getSwerveModule(int moduleNumber) {
            // Belirtilen modül pozisyonuna göre ilgili modülü döndürür
            return this.m_swerveModulesToPositions.get(ModulePosition.values()[moduleNumber]);
        }

        /**
         * @param position Modülün {@link ModulePosition pozisyonu}
         * @return Belirtilen pozisyondaki {@link SwerveModule swerve modülü}
         */
        public SwerveModule getSwerveModule(ModulePosition position) {
            // Belirtilen pozisyona göre ilgili modülü döndürür
            return this.m_swerveModulesToPositions.get(position);
        }

        // Saha yönelimi ile ilgili yöntemler
        /** @return Robotun saha yönelimi modunda olup olmadığını döndürür */
        public boolean isFieldCentric() {
            return isFieldCentric;
        }

        /** @return isFieldCentric() yönteminin tersini döndürür */
        public boolean isRobotCentric() {
            return !isFieldCentric;
        }

        /**
         * @param isFieldCentric Robotun saha yönelimi modunda olup olmayacağını ayarlar
         */
        public void setFieldCentric(boolean isFieldCentric) {
            this.isFieldCentric = isFieldCentric;
        }

        /**
         * @param isFieldCentric Robotun robot yönelimi modunda olup olmayacağını
         *                       ayarlar
         */
        public void setRobotCentric(boolean isRobotCentric) {
            this.isFieldCentric = !isRobotCentric;
        }

        /**
         * Robotu saha yönelimi modundan robot yönelimi moduna ya da tam tersi moda
         * geçirir
         */
        public void toggleFieldCentric() {
            this.isFieldCentric = !this.isFieldCentric;
        }

        /** Robotun tekerleklerini ileriye doğru hizalar */
        public void zeroWheels() {
            // Her bir modülü sıfır hız ve sıfır açı ile hizalar
            for (SwerveModule module : this.m_swerveModulesToPositions.values())
                module.setDesiredState(
                        new SwerveModuleState(0, new Rotation2d(0)),
                        true);
        }

        /**
         * @return Her modülün mevcut {@link SwerveModuleState durumunu} içeren bir dizi
         *         döndürür
         */
        public SwerveModuleState[] getModuleStates() {
            return new SwerveModuleState[] {
                    this.m_swerveModulesToPositions.get(ModulePosition.FL).getState(),
                    this.m_swerveModulesToPositions.get(ModulePosition.FR).getState(),
                    this.m_swerveModulesToPositions.get(ModulePosition.RL).getState(),
                    this.m_swerveModulesToPositions.get(ModulePosition.RR).getState()
            };
        }

        /**
         * Her bir modülün durumunu aynı anda ayarlar
         * 
         * @param states Her modülün istenen {@link SwerveModuleState durumu}
         *               içeren bir dizi
         */
        public void setModuleStates(SwerveModuleState[] states) {
            // Eğer modüller maksimum hızdan daha hızlı hareket edecekse hızı normalize eder
            SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MaxSpeeds.kTranslation);

            // Her modüle talimatları gönderir
            for (SwerveModule module : this.m_swerveModulesToPositions.values())
                module.setDesiredState(states[module.getModuleNumber()], true);
        }

        /** Tüm modüllerin durumunu sıfırlar */
        public void resetModuleStates() {
            // Her modüle sıfır hız ve sıfır açı gönderir
            for (SwerveModule module : this.m_swerveModulesToPositions.values())
                module.setDesiredState(new SwerveModuleState(), true);
        }

        /**
         * @return Her modülün mevcut {@link SwerveModulePosition pozisyonunu}
         *         içeren bir dizi döndürür
         */
        public SwerveModulePosition[] getModulePositions() {
            return new SwerveModulePosition[] {
                    this.m_swerveModulesToPositions.get(ModulePosition.FL).getPosition(),
                    this.m_swerveModulesToPositions.get(ModulePosition.FR).getPosition(),
                    this.m_swerveModulesToPositions.get(ModulePosition.RL).getPosition(),
                    this.m_swerveModulesToPositions.get(ModulePosition.RR).getPosition()
            };
        }

        /**
         * Her modülün {@link SwerveModulePosition pozisyonu} ve robotun mevcut baş
         * açısı
         * kullanılarak robotun odometrisini günceller
         */
        public void updateOdometry() {
            // Odometriyi günceller
            this.m_odometry.update(getHeadingRotation2d(), getModulePositions());

            // Her bir modülün sahadaki pozisyonunu hesaplar ve ayarlar
            for (SwerveModule module : this.m_swerveModulesToPositions.values()) {
                var modulePositionFromChassis = Constants.Swerve.kModuleTranslations[module.getModuleNumber()]
                        .rotateBy(getHeadingRotation2d())
                        .plus(getPoseMeters().getTranslation());
                module.setModulePose(
                        new Pose2d(
                                modulePositionFromChassis,
                                module.getHeadingRotation2d().plus(getHeadingRotation2d())));
            }
        }

        /**
         * Robotun odometrisini verilen pozisyon kullanarak ayarlar
         * 
         * @param pose Robotun pozisyonu
         */
        public void setOdometry(Pose2d pose) {
            // Odometriyi verilen pozisyona sıfırlar
            this.m_odometry.resetPosition(
                    getHeadingRotation2d(),
                    getModulePositions(),
                    pose);
        }

        /** Robotun odometrisini sıfırlar */
        public void resetOdometry() {
            // Odometriyi sıfır pozisyona sıfırlar
            this.m_odometry.resetPosition(
                    getHeadingRotation2d(),
                    getModulePositions(),
                    new Pose2d());
        }

        @Override // Her 20 ms'de bir çağrılır
        public void periodic() {
            updateOdometry();
            // SmartDashboard'a robotun baş açısını yazdırır
            SmartDashboard.putNumber("Heading in Degrees", getHeadingDegrees());
            
            // SmartDashboard'a robotun saha modunda mı yoksa robot modunda mı olduğunu
            // yazdırır
        
        }
    }