package org.firstinspires.ftc.teamcode.testing;

public class MecDT extends DT{
    private double transScl;
    private double rotScl;

    void setScls(double trans, double rot){
        transScl = trans;
        rotScl = rot;
    }
    void directionalPow(double forward, double side, double rot){
        motorPow(
                (forward + side)*transScl - rot*rotScl,
                (forward - side)*transScl + rot*rotScl,
                (forward - side)*transScl - rot*rotScl,
                (forward + side)*transScl + rot*rotScl
        );
    }
    void absoluteDirectionalPow(double robotDir, double forward, double side, double rot){
        // Visualization of why this works: https://www.desmos.com/calculator/kgy249h8rj
        directionalPow(
                forward*Math.cos(robotDir) + side*Math.sin(robotDir),
                forward*Math.cos(robotDir + Math.PI/2) + side*Math.sin(robotDir + Math.PI/2),
                rot
        );
    }
}
