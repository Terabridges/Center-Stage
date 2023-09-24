package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private double transScl = 1./3.;
    private double rotScl = 1./3.;

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
        setDirectionsStandard();
    }
    void setDirections(DcMotorSimple.Direction fl, DcMotorSimple.Direction fr, DcMotorSimple.Direction bl, DcMotorSimple.Direction br){
        this.fl.setDirection(fl);
        this.fr.setDirection(fr);
        this.bl.setDirection(bl);
        this.br.setDirection(br);
    }
    void setDirectionsStandard(){
        setDirections(
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        );
    }

    DcMotor getFl() {
        return fl;
    }
    DcMotor getFr() {
        return fr;
    }
    DcMotor getBl() {
        return bl;
    }
    DcMotor getBr() {
        return br;
    }

    void motorPow(double fl, double fr, double bl, double br){
        double max = Math.max(
                1.,
                Math.max(
                        Math.max(
                                fl,
                                fr
                        ),
                        Math.max(
                                bl,
                                br
                        )
                )
        );
        this.fl.setPower(fl/max);
        this.fr.setPower(fr/max);
        this.bl.setPower(bl/max);
        this.br.setPower(br/max);
    }
    void transRotPow(double forward, double side, double rot){
        motorPow(
                (forward - side)*transScl - rot*rotScl,
                (forward + side)*transScl + rot*rotScl,
                (forward + side)*transScl - rot*rotScl,
                (forward - side)*transScl + rot*rotScl
        );
    }
    void absoluteTransRotPow(double robotDir, double forward, double side, double rot){
        // Visualization of why this works: https://www.desmos.com/calculator/w7o0jrr7fq
        transRotPow(
                forward*Math.cos(robotDir) + side*Math.sin(robotDir),
                forward*Math.cos(robotDir + Math.PI/2) + side*Math.sin(robotDir + Math.PI/2),
                rot
        );
    }
}
