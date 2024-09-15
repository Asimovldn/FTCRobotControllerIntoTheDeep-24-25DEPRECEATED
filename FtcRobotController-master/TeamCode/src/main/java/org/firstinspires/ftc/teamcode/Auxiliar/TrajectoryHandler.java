package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TrajectoryHandler
{
    IMU imu;

    YawPitchRollAngles robotOrientation;

    public  TrajectoryHandler(IMU imu)
    {
        this.imu = imu;

        robotOrientation = imu.getRobotYawPitchRollAngles();
    }

    public void moveOnStraightLine(Vector2D distance)
    {
        double currentYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        Vector2D robotRelativeDirection = Vector2D.normalizeVector(Vector2D.rotateVector(distance, -currentYaw));

        double distanceMagnitude = distance.magnitude;

        TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(DriveConstants.maxVel, DriveConstants.maxAccel, distanceMagnitude);

        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while (motionProfile.isBusy)
        {
            double[] motionProfileValues = motionProfile.calculateMotionProfile(elapsedTime.time());

            Vector2D robotVelocity = Vector2D.scalarMultiplication(robotRelativeDirection, motionProfileValues[1]);
            Vector2D robotAccel = Vector2D.scalarMultiplication(robotRelativeDirection, motionProfileValues[2]);
        }


    }

}
