package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private double transScl;
    private double rotScl;

    Drivetrain(){
        fl = null;
        fr = null;
        bl = null;
        br = null;
    }

    Drivetrain(float trans, float rot){
        this();
        setScls(trans, rot);
    }

    void setScls(float trans, float rot){
        transScl = trans;
        rotScl = rot;
    }

    void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }
    void setMotors(HardwareMap hardwareMap){
        setMotors(
                hardwareMap.get(DcMotor.class, "leftfront_drive"),
                hardwareMap.get(DcMotor.class, "rightfront_drive"),
                hardwareMap.get(DcMotor.class, "leftback_drive"),
                hardwareMap.get(DcMotor.class, "rightback_drive")
        );
    }

    void motorPow(double fl, double fr, double bl, double br){
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.bl.setPower(bl);
        this.br.setPower(br);
    }
    void transRotPow(double forward, double side, double rot){
        motorPow(
                (forward + side)*transScl - rot*rotScl,
                (forward - side)*transScl + rot*rotScl,
                (forward - side)*transScl - rot*rotScl,
                (forward + side)*transScl + rot*rotScl
        );
    }
    void absoluteTransRotPow(double robotDir, double forward, double side, double rot){
        // Visualization of why this works: https://www.desmos.com/calculator/kgy249h8rj
        transRotPow(
                forward*Math.cos(robotDir) + side*Math.sin(robotDir),
                forward*Math.cos(robotDir + Math.PI/2) + side*Math.sin(robotDir + Math.PI/2),
                rot
        );
    }
}
