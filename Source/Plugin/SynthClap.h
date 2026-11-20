// SynthClap.h
// The three entry functions clap-wrapper links into every format.

#pragma once

bool tbsynth_entry_init (const char* pluginPath);
void tbsynth_entry_deinit();
const void* tbsynth_entry_get_factory (const char* factoryId);
