package org.firstinspires.ftc.teamcode.Auxiliar;

public class MecannumKinematics
{

    // recebe na ordem: esquerda frontal, direita frontal, direita trazeira, esquerda trazeira

    // retorna na ordem: vy, vx, angular
    public static double[] forwardKinematics(double[] wheelVelocities)
    {
        double vy = DriveConstants.wheelRadius * (wheelVelocities[0] + wheelVelocities[1] +
                wheelVelocities[2] + wheelVelocities[3]) / 4;

        double vx = DriveConstants.wheelRadius * (-wheelVelocities[0] + wheelVelocities[1]
                - wheelVelocities[2] + wheelVelocities[3]) / 4;

        double angularVelocity = DriveConstants.wheelRadius * (-wheelVelocities[0] + wheelVelocities[1]
                + wheelVelocities[2] - wheelVelocities[3]);

        return new double[] {vy, vx, angularVelocity};
    }

    // recebe na ordem: vy, vx, angular
    // returna na ordem: esquerda frontal, direita frontal, direita trazeira, esquerda trazeira

    public static double[] inverseKinematics(double[] robotVelocities)
    {
        double frontLeft = (robotVelocities[0] - robotVelocities[1] -
                DriveConstants.trackWidth * robotVelocities[2]) / DriveConstants.wheelRadius;

        double frontRight = (robotVelocities[0] + robotVelocities[1] +
                DriveConstants.trackWidth * robotVelocities[2]) / DriveConstants.wheelRadius;

        double backRight = (robotVelocities[0] - robotVelocities[1] +
                DriveConstants.trackWidth * robotVelocities[2]) / DriveConstants.wheelRadius;

        double backLeft = (robotVelocities[0] + robotVelocities[1] -
                DriveConstants.trackWidth * robotVelocities[2]) / DriveConstants.wheelRadius;

        return new double[] {frontLeft, frontRight, backRight, backLeft};
    }

}

