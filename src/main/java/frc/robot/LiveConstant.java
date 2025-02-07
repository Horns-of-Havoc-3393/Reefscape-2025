package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class LiveConstant {
    public class LNumber extends LoggedNetworkNumber {
        LNumber(double initVal, String key) {
            super(key);
            this.set(initVal);
        }
    }
    public class LBoolean extends LoggedNetworkBoolean {
        LBoolean(boolean initVal, String key) {
            super(key);
            this.set(initVal);
        }
    }
    public class LString extends LoggedNetworkString {
        LString(String initVal, String key) {
            super(key);
            this.set(initVal);
        }
    }
}
