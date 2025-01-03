package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/*PID Swerve de ne işe yarıyor. Sabitleme
P (Proportional - Orantısal): Anlık hata ile orantılı bir düzeltme yapar. Tekerlek istenen pozisyona ne kadar uzaktaysa, düzeltme sinyali o kadar büyük olur.
I (Integral - Tümevarım): Hataların zamanla birikimini düzeltir. Sistem küçük bir hata yapmaya devam ediyorsa, bunu zaman içinde giderir.
D (Derivative - Türev): Hatanın değişim hızını dikkate alarak sistemin ani değişimlere aşırı tepki vermesini önler.*/


// Sabitlerin tanımlandığı Constants sınıfı
public final class Constants {

    // Kontrol cihazları için sabitler
    public static final class Controllers {

        // Sürücü kontrol cihazı için sabitler
        public static final class Driver {
            public static final int kPort = 0;  // Sürücü kontrol cihazının bağlantı portu (USB port numarası)
            public static final double kTranslationDeadband = 0.05;  // Çeviri (ileri-geri) hareketi için deadband (ölü bölge) değeri. 
            public static final double kRotationDeadband = 0.1;  // Dönme hareketi için deadband değeri
        }
    }

    // Swerve sürüş sistemi ile ilgili sabitler
    public static final class Swerve {

        // Maksimum hızlarla ilgili sabitler
        public static final class MaxSpeeds {
            public static final double kTranslation = 4.5; // m/s, robotun doğrusal (ileri-geri) hareketinin maksimum hızı
            public static final double kRotation = Math.PI; // rad/s, robotun dönüş hızının maksimum değeri
        }

        // Sürüş hareketlerinin yavaşlama (slew rate) limitleri
        public static final class SlewRateLimits {
            public static final double kForwardsSlewRateLimit = 2; // İleri hareket için hız değişim limiti (m/s^2)
            public static final double kStrafeSlewRateLimit = 2; // Yan hareket için hız değişim limiti (m/s^2)
            public static final double kRotateSlewRateLimit = 3; // Dönüş için hız değişim limiti (rad/s^2)
        }

        // Motor akım limitleri
        public static final class CurrentLimits {
            public static final int kDriveMotor = 60; // Sürüş motoru için akım limiti (amper)
            public static final int kAngleMotor = 20; // Açı motoru için akım limiti (amper)
        }

        // Sürüş sistemi ile ilgili sabitler
        public static final class Drive {
            public static final double kGearRatio = (36.0 / 14.0) * (18.0 / 24.0) * (45.0 / 15.0); // Dişli oranı
            public static final double kWheelDiameter = 0.096706557; // Tekerlek çapı (metre cinsinden)
            public static final double kRevolutionsToMeters = ((kWheelDiameter * Math.PI) / kGearRatio); // Dönüşü metreye çevirme faktörü
            public static final double kRPMtoMPS = kRevolutionsToMeters / 60.0; // Devir/dakika (RPM)'yi metre/saniye (MPS)'ye çevirme
            
            
            //public static final double kTrackWidth = Units.inchesToMeters(25.5); // Robotun tekerlekler arasındaki mesafe (metre cinsinden)
            //public static final double kWheelBase = Units.inchesToMeters(25.5); // Robotun tekerlek tabanı uzunluğu (metre cinsinden)
            public static final double kTrackWidth = 0.56; // Metre cinsinden direkt değer
            public static final double kWheelBase = 0.56; // Metre cinsinden direkt değer
        }

        // Açı kontrolü ile ilgili sabitler
        public static final class Angle {
            public static final double kP = 0.005; // Açı kontrolü için PID denetleyicisi P katsayısı
            public static final double kGearRatio = (50.0 / 14.0) * (72.0 / 14.0); // Açı motoru için dişli oranı
            public static final double kRevolutionsToDegrees = 360.0 / kGearRatio; // Dönüşü dereceye çevirme faktörü
        }

        // Modül konumları için vektörler (x, y koordinatları)
        //Translation2d: kordinatı her tekerleğin NAVX'e yani robotun ortasına gore konumunu belli eder
        //Drive.kWheelBase / 2.0: Tekerleğin robot merkezine olan mesafesini temsil eder (yönü x ekseninde belirler).
        //Drive.kTrackWidth / 2.0: Tekerleğin merkezden sağa veya sola olan mesafesini temsil eder (yönü y ekseninde belirler).
        //Tekerlek: Sağ ön (pozitif x, pozitif y).
        //Tekerlek: Sol ön (pozitif x, negatif y).
        //Tekerlek: Sağ arka (negatif x, pozitif y).
        //Tekerlek: Sol arka (negatif x, negatif y).
        //Drive.kWheelBase: Robotun ön-arka tekerlekleri arasındaki mesafe (uzunluk).
        //Drive.kTrackWidth: Robotun sağ-sol tekerlekleri arasındaki mesafe (genişlik).
        //2.0 da yarısını almak için
        public static final Translation2d[] kModuleTranslations = {
                new Translation2d(Drive.kWheelBase / 2.0, Drive.kTrackWidth / 2.0),
                new Translation2d(Drive.kWheelBase / 2.0, -Drive.kTrackWidth / 2.0),
                new Translation2d(-Drive.kWheelBase / 2.0, Drive.kTrackWidth / 2.0),
                new Translation2d(-Drive.kWheelBase / 2.0, -Drive.kTrackWidth / 2.0)
        };

        // Swerve sürüş kinematiği hesaplamaları
        public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);

        // Ön sol modül (FL) için sabitler
        public static final class FL {
            public static final int kDriveMotorID = 11; // Sürüş motoru ID'si
            public static final int kAngleMotorID = 12; // Açı motoru ID'si
            public static final boolean kDriveInvert = true; // Sürüş motoru ters mi? (False: doğru, True: ters)
            public static final boolean kAngleInvert = false; // Açı motoru ters mi? (False: doğru, True: ters)
            public static final int kAngleAbsoluteEncoderID = 1; // Açı mutlak enkoder ID'si
            public static final double kAngleAbsoluteEncoderOffset = 5; // Açı enkoder ofseti
        }

        // Ön sağ modül (FR) için sabitler
        public static final class FR {
            public static final int kDriveMotorID = 31;
            public static final int kAngleMotorID = 32;
            public static final boolean kDriveInvert = false;
            public static final boolean kAngleInvert = false;
            public static final int kAngleAbsoluteEncoderID = 3;
            public static final double kAngleAbsoluteEncoderOffset = -206.73;
        }

        // Arka sol modül (RL) için sabitler
        public static final class RL {
            public static final int kDriveMotorID = 21;
            public static final int kAngleMotorID = 22;
            public static final boolean kDriveInvert = true;
            public static final boolean kAngleInvert = false;
            public static final int kAngleAbsoluteEncoderID = 2;
            public static final double kAngleAbsoluteEncoderOffset = 10;
        }

        // Arka sağ modül (RR) için sabitler
        public static final class RR {
            public static final int kDriveMotorID = 41;
            public static final int kAngleMotorID = 42;
            public static final boolean kDriveInvert = false;
            public static final boolean kAngleInvert = false;
            public static final int kAngleAbsoluteEncoderID = 4;
            public static final double kAngleAbsoluteEncoderOffset = 30;
        }

        // Modül konumu enum (FL, FR, RL, RR) tanımlaması
        public enum ModulePosition {
            FL,  // Ön sol modül
            FR,  // Ön sağ modül
            RL,  // Arka sol modül
            RR   // Arka sağ modül
;

            public double FL() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'FL'");
            }
        }
    }
}