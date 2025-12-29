package org.firstinspires.ftc.teamcode.junit;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;

import static org.junit.jupiter.api.Assertions.*;

import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.Arrays;

public class SampleTest {
    @Test
    public void example() {
        InterpLUT launcherLUT = new InterpLUT(
                Arrays.asList(-0.01, 0.0, 4.29,   4.76,   5.22,   5.65,   6.06,   6.48,   10.0), // input: velocity (m/s)
                Arrays.asList(0.0,   0.0, 1267.0, 1367.0, 1500.0, 1667.0, 1790.0, 1967.0, 2000.0), // output: ticks/s
                true
        );

        System.out.println(launcherLUT.get(-10.0));

        int x = 2;
        assertEquals(2, x);
    }
}
