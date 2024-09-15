package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

// Esse arquivo salva constantes que serão utilizadas diversas vezes em vários arquivos diferentes
public class DriveConstants
{
    public static IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
    );

    public static double trackWidth = 0; // cm
    public static double wheelRadius = 9.6; // cm
    public static double motorTicksPerRev = 28; // ticks por volta
    public static double gearReduction = 1.0 / 20.0; // Output / Input


    public static double maxVel = 50; // cm/s
    public static double maxAccel = 50; // cm/s^2

}
