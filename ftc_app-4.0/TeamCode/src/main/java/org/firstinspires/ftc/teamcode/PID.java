package org.firstinspires.ftc.teamcode;

/**
 * Created by grant on 2/18/2018.
 *
 * PID.java
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//this is some pretty complicated PID stuff that I personally dont understand well.
//This class is a library (lib) that is called on in some of our autonomous programs for PID turning.
//PID is a highly accurate (albeit difficult and often frustrating) way to program precise autonomous driving (especially turning functions)
public class PID {
    ElapsedTime timer;

    public double Kp, Ki, Kd;
    public double target;
    public double prevError;
    public double iTerm;
    public double dTerm;
    public double iThresh;
    public double targetThresh;
    public double heading;
    public int check;
    public int count = 3;


    public PID(double Kp, double Ki, double Kd, double iThresh, double targetThresh) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.iThresh = iThresh;
        this.targetThresh = targetThresh;
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void setTarget(int target){
        this.target = target;
        if (target == 0) {
            this.target = 360;
        }
        check = 0;
        timer.reset();
        iTerm = 0;
        prevError = -1;
    }

    public double getOutput(double h) {
        double dt = timer.time();

        heading = h;

        if (target == 90) {
            if (heading > 180) heading -= 360;
        } else if (target == 360) {
            if (heading < 180) heading += 360;
        } else if (target == 270) {
            if (heading < 180) heading += 360;
        }

        double error = target - heading;

        // sets up P
        if (prevError == -1) {
            prevError = error;
        }

        // sets up I
        if (Math.abs(error) <= iThresh || iThresh == -1) {
            iTerm += error * dt;
        }

        // sets up D
        dTerm = (error - prevError)/dt;

        // output calculated
        double output = Kp * error + Ki * iTerm + Kd * dTerm;
        output = Range.clip(output, -1, 1);

        prevError = error;
        timer.reset();
        return output * -1;
    }

    public boolean isAtTarget() {
        if (prevError == -1) {
            return false;
        }

        if (targetThresh >= Math.abs(prevError)) {
            check += 1;
        }

        if (check < count) {
            return false;
        } else {
            return true;
        }
    }
}