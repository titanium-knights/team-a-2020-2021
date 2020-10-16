package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadManager {

    public class GMButton {

        private boolean lastPos;
        private boolean currPos;

        protected void setPos (boolean down) {
            lastPos = currPos;
            currPos = down;
        }

        public boolean getPos () {
            return currPos;
        }

        public boolean pressed () { return !lastPos && currPos; }
        public boolean released () { return lastPos && !currPos; }

    }

    private Gamepad gamepad;

    public GMButton a, b, x, y, left_bumper, right_bumper;

    public GamepadManager (Gamepad gamepad) {
        this.gamepad = gamepad;

        a = new GMButton();
        b = new GMButton();
        x = new GMButton();
        y = new GMButton();
        left_bumper = new GMButton();
        right_bumper = new GMButton();
    }

    public void updateAll() {
        a.setPos(gamepad.a);
        b.setPos(gamepad.b);
        x.setPos(gamepad.x);
        y.setPos(gamepad.y);
        left_bumper.setPos(gamepad.left_bumper);
        right_bumper.setPos(gamepad.right_bumper);
    }

}