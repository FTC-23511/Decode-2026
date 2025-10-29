package dev.nullftc.profiler.entry;

import dev.nullftc.profiler.entry.ProfilerEntry;

public interface ProfilerEntryFactory {
    ProfilerEntry create(String type, long start, long end);
}
