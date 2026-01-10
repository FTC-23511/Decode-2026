package org.firstinspires.ftc.teamcode.junit;

import org.junit.jupiter.api.Test;

import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_ANGULAR_VELOCITY;
import static org.junit.jupiter.api.Assertions.*;

import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.Arrays;

public class LauncherMathTest {
    @Test
    public void lutTest() {
        InterpLUT launcherLUT = new InterpLUT(
                Arrays.asList(-0.01, 0.0, 4.29,   4.76,   5.22,   5.65,   6.06,   6.48,   10.0), // input: velocity (m/s)
                Arrays.asList(0.0,   0.0, 1267.0, 1367.0, 1500.0, 1667.0, 1790.0, 1967.0, 2000.0), // output: ticks/s
                true
        );

        double val = launcherLUT.get(-10.0);

        System.out.println("launcherLut: " + MAX_ANGULAR_VELOCITY);

        assertEquals(0.0, val);
    }
}
