package com.seattlesolvers.solverslib.hardware.servos;

import androidx.annotation.NonNull;

import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Allows multiple {@link CRServo} objects to be linked together
 * as a single group. Multiple CRServo's will act together.
 * Based off {@link MotorGroup}, but for CRServo/CRServoEx
 *
 * @author Arush
 * @author Jackson
 */
public class ServoExGroup extends ServoEx implements Iterable<ServoEx> {

    private final ServoEx[] group;

    /**
     * Create a new CRServoGroup with the provided CRServos.
     *
     * @param leader    The leader CRServo.
     * @param followers The follower CRServos which follow the leader CRServo's protocols.
     */
    public ServoExGroup(@NonNull ServoEx leader, ServoEx... followers) {
        group = new ServoEx[followers.length + 1];
        group[0] = leader;
        System.arraycopy(followers, 0, group, 1, followers.length);
    }

    /**
     * Set the position for each CRServo in the group
     *
     * @param position The position to set. Value should be between -1.0 and 1.0.
     */
    @Override
    public void set(double position) {
        group[0].set(position);
        for (int i = 1; i < group.length; i++) {
            group[i].set(group[0].get());
        }
    }

    /**
     * @return The last position
     */
    @Override
    public double get() {
        return group[0].get();
    }

    /**
     * @return All CRServo target speeds as a percentage of output
     */
    public List<Double> getPositions() {
        return Arrays.stream(group)
                .map(ServoEx::get)
                .collect(Collectors.toList());
    }

    @NonNull
    @Override
    public Iterator<ServoEx> iterator() {
        return Arrays.asList(group).iterator();
    }

    /**
     * @return true if the CRServo group is inverted
     */
    @Override
    public boolean getInverted() {
        return group[0].getInverted();
    }

    /**
     * Set the CRServo group to the inverted direction or forward direction.
     *
     * @param isInverted The state of inversion true is inverted.
     * @return This object for chaining purposes.
     */
    public ServoExGroup setInverted(boolean isInverted) {
        for (ServoEx servo : group) {
            servo.setInverted(isInverted);
        }
        return this;
    }

    /**
     * Disables all the Servo devices.
     */
    public void disable() {
        for (ServoEx servo : group) {
            servo.disable();
        }
    }

    /**
     * @return a string characterizing the device type
     */
    public String getDeviceType() {
        return "Servo Group";
    }
}
