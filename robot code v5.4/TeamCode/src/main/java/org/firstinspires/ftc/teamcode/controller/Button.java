package org.firstinspires.ftc.teamcode.controller;

public class Button {
    private boolean state;
    private boolean prevState;

    public Button() {
        state = false;
        prevState = false;
    }

    public void setState(boolean state){
        this.state = state;
    }

    public void setPrevState(){
        this.prevState = this.state;
    }

    public boolean isPressed(){
        return state;
    }

    public boolean onClick(){
        return this.state && !this.prevState;
    }

    public boolean onRelease(){
        return !this.state && this.prevState;
    }
}
