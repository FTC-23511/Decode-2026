package dev.nullftc.profiler.entry;

import dev.nullftc.profiler.entry.BasicProfilerEntry;
import dev.nullftc.profiler.entry.ProfilerEntry;
import dev.nullftc.profiler.entry.ProfilerEntryFactory;

public class BasicProfilerEntryFactory implements ProfilerEntryFactory {
    @Override
    public ProfilerEntry create(String type, long start, long end) {
        return new BasicProfilerEntry(type, start, end);
    }
}
