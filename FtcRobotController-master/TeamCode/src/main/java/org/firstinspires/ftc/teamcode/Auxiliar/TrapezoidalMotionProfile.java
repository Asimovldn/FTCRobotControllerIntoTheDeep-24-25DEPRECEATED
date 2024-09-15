package org.firstinspires.ftc.teamcode.Auxiliar;

public class TrapezoidalMotionProfile
{
    double maxVel;
    double maxAccel;
    double distance;

    double accelDistance;
    double cruiseDistance;

    double endAccelTimeStamp;
    double endCruiseTimeStamp;
    double totalTime;

    public boolean isBusy;

    public TrapezoidalMotionProfile(double maxVel, double maxAccel, double distance)
    {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.distance = distance;

        double timeToAccel = this.maxVel / maxAccel;

        accelDistance = 0.5 * maxAccel * Math.pow(timeToAccel, 2);

        if (accelDistance > distance / 2)
        {
            timeToAccel = Math.sqrt(distance / maxAccel);
        }

        accelDistance = 0.5 * maxAccel * Math.pow(timeToAccel, 2);

        this.maxVel = maxAccel * timeToAccel;

        cruiseDistance = distance - 2 * accelDistance;
        double cruiseTime = cruiseDistance / this.maxVel;

        endAccelTimeStamp = timeToAccel;
        endCruiseTimeStamp = timeToAccel + cruiseTime;

        totalTime = 2 * timeToAccel + cruiseTime;


    }


    // Retorna na ordem: distancia atual, velocidade atual, aceleração atual
    public double[] calculateMotionProfile(double time)
    {
        if (time < totalTime)
        {
            isBusy = true;
        }

        if (!isBusy)
        {
            return new double[] {0,0,0};
        }

        double currentDistance = 0.0;
        double currentVelocity = 0.0;
        double currentAccel = 0.0;

        if (time < endAccelTimeStamp)
        {
            // Acelerando
            currentAccel = maxAccel;
            currentVelocity = maxAccel * time;
            currentDistance = 0.5 * maxAccel * Math.pow(time, 2);
        } else if (time < endCruiseTimeStamp) {
            // Cruzeiro
            currentAccel = 0.0;
            currentVelocity = maxVel;
            currentDistance = accelDistance + maxVel * (time - endAccelTimeStamp);
        } else if (time < totalTime) {
            // Desacelerando
            currentAccel = -maxAccel;
            currentVelocity = maxVel - maxAccel * (time - endCruiseTimeStamp);
            currentDistance = accelDistance + cruiseDistance + maxVel * (time - endCruiseTimeStamp)
                    - 0.5 * maxAccel * Math.pow(time - endCruiseTimeStamp, 2);
        }

        return new double[] {currentDistance, currentVelocity, currentAccel};
    }


    /*
            double motion_profile(max_acceleration, max_velocity, distance, elapsed_time) {
          """
          Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
          """

          // Calculate the time it takes to accelerate to max velocity
          acceleration_dt = max_velocity / max_acceleration

          // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
          halfway_distance = distance / 2
          acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2

          if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration))
          }

          acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2

          // recalculate max velocity based on the time we have to accelerate and decelerate
          max_velocity = max_acceleration * acceleration_dt

          // we decelerate at the same rate as we accelerate
          deceleration_dt = acceleration_dt

          // calculate the time that we're at max velocity
          cruise_distance = distance - 2 * acceleration_distance
          cruise_dt = cruise_distance / max_velocity
          deceleration_time = acceleration_dt + cruise_dt

          // check if we're still in the motion profile
          entire_dt = acceleration_dt + cruise_dt + deceleration_dt
          if (elapsed_time > entire_dt) {
            return distance
          }

          // if we're accelerating
          if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * elapsed_time ** 2
          }

          // if we're cruising
          else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
            cruise_current_dt = elapsed_time - acceleration_dt

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt
          }

          // if we're decelerating
          else {
            acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
            cruise_distance = max_velocity * cruise_dt
            deceleration_time = elapsed_time - deceleration_time

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time ** 2
          }
        }
     */
}
