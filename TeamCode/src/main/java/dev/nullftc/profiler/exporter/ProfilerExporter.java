package dev.nullftc.profiler.exporter;

import java.util.List;

import dev.nullftc.profiler.entry.ProfilerEntry;

public interface ProfilerExporter {
    void export(List<ProfilerEntry> entries);
}
