package org.firstinspires.ftc.teamcode.junit;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;

import static org.junit.jupiter.api.Assertions.*;

public class SampleTest {

    @BeforeEach
    public void setUp() {
        // Setup code before each test
    }

    @AfterEach
    public void tearDown() {
        // Cleanup code after each test
    }

    @Test
    public void example() {
        double x = 1 + 1;
        assertEquals(2, x);
    }
}
