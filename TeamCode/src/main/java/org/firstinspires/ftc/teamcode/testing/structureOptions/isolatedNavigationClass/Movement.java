package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;

// Represents a translational movement followed by a rotational movement.
public class Movement {
    private double side = 0;
    private double forward = 0;
    private double rot = 0;

    Movement(double side, double forward, double rot){
        set(side, forward, rot);
    }
    Movement(Movement position){
        set(position);
    }
    Movement(double side, double forward){
        set(side, forward);
    }
    Movement(){}

    void setSide(double side){
        this.side = side;
    }
    void setForward(double forward){
        this.forward = forward;
    }
    void setRot(double rot){
        this.rot = rot;
    }
    void set(double side, double forward, double rot){
        setSide(side);
        setForward(forward);
        setRot(rot);
    }
    void set(double side, double forward){
        this.side = side;
        this.forward = forward;
    }
    void set(Movement movement){
        set(
                movement.getSide(),
                movement.getForward(),
                movement.getRot()
        );
    }

    double getSide(){
        return side;
    }
    double getForward(){
        return forward;
    }
    double getRot(){
        return rot;
    }

    void add(double side, double forward, double rot){
        this.side += side*Math.cos(this.rot) + forward*Math.cos(this.rot + Math.PI/2);
        this.forward += side*Math.sin(this.rot) + forward*Math.sin(this.rot + Math.PI/2);
        this.rot += rot;
    }
    void add(Movement movement){
        add(
                movement.getSide(),
                movement.getForward(),
                movement.getRot()
        );
    }

    void remove(double side, double forward, double rot){
        this.rot -= rot;
        this.side -= side*Math.cos(this.rot) + forward*Math.cos(this.rot + Math.PI/2);
        this.forward -= side*Math.sin(this.rot) + forward*Math.sin(this.rot + Math.PI/2);
    }
    void remove(Movement movement){
        remove(
                movement.getSide(),
                movement.getForward(),
                movement.getRot()
        );
    }

    Movement solve(double side, double forward, double rot){
        return new Movement(
                (this.side - side)*Math.cos(this.rot) - (this.forward - forward)*Math.sin(this.rot),
                (this.side - side)*Math.sin(this.rot) + (this.forward - forward)*Math.cos(this.rot),
                rot - this.rot
        );
    }
    Movement solve(Movement movement){
        return solve(
                movement.getSide(),
                movement.getForward(),
                movement.getRot()
        );
    }

    double distance(double side, double forward){
        return Math.sqrt(Math.pow(this.side - side, 2) + Math.pow((this.forward - forward), 2));
    }
    double distance(double side, double forward, double rot){
        return distance(side, forward);
    }
    double distance(Movement movement){
        return distance(
                movement.getSide(),
                movement.getForward()
        );
    }
}
