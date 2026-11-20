// SynthClap.cpp
// TB Synth as a CLAP instrument. clap-wrapper builds the VST3 and AU versions
// from this same code, so all three formats behave identically. The DSP, the
// presets and the panel are the Mountain Lion build's, unchanged.
//
// - Notes in as CLAP note events or MIDI (notes, pitch bend, mod wheel, sustain
//   pedal, all notes off), played at their exact sample in the block.
// - Mono, like the originals: the same signal on both outputs.
// - Every control is a host parameter, so a session reopens exactly as it was.
//
// Threads:
//  - Audio thread: process() / flush(). Only these touch the DSP while active.
//  - Main thread: parameter info, state, the panel and the preset store.
//  Parameter values live in atomics both sides can read. Changes from the panel
//  (or a loaded state) travel to the audio thread through a lock-free FIFO and
//  are reported to the host as gesture / value events.

#include <clap/clap.h>

#include "Editor.h"
#include "Format.h"
#include "Presets.h"
#include "SynthClap.h"
#include "../DSP/Synth.h"

#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <strings.h>

#if defined(__SSE__) || defined(__x86_64__) || defined(_M_X64)
  #include <xmmintrin.h>
  #define TBSYN_HAVE_SSE 1
#endif

using namespace tbsyn;

namespace
{

const char* const kPluginId = "com.tonybuzzcock.tb-synth";

const char* const kFeatures[] = { CLAP_PLUGIN_FEATURE_INSTRUMENT, CLAP_PLUGIN_FEATURE_SYNTHESIZER,
                                  CLAP_PLUGIN_FEATURE_MONO, nullptr };

const clap_plugin_descriptor_t kDescriptor = {
    CLAP_VERSION_INIT,
    kPluginId,
    "TB Synth",
    "TB",
    "https://github.com/tonybuzzcock/TB-Synth-2026-MAC",
    "",
    "",
    "1.0.0",
    "Twelve classic keyboards in one: Minimoog, OB-X, Solina, Wurlitzer, B-3 with Leslie, Cembalet, Clavinet, "
    "Multiman, Omni, SK-30, Rhapsody 610, Moog Modular",
    kFeatures
};

// Values the host sees: switches and selectors as their position number (the
// INSTRUMENT as 0..11), everything else 0..1 as the panel's knobs
int plainSteps (int i) { return i == kInst ? kNumInstruments : kParams[i].steps; }
bool isStepped (int i) { return plainSteps (i) >= 2; }
double plainMax (int i) { return isStepped (i) ? (double) (plainSteps (i) - 1) : 1.0; }

double toPlain (int i, float internal)
{
    if (i == kInst) return (double) instIndex (internal);
    if (isStepped (i)) return (double) stepOf (internal, kParams[i].steps);
    return internal;
}

float fromPlain (int i, double plain)
{
    if (! (plain == plain)) return kParams[i].def;
    const double hi = plainMax (i);
    if (plain < 0.0) plain = 0.0;
    if (plain > hi) plain = hi;
    if (i == kInst) return instValue ((int) std::floor (plain + 0.5));
    if (isStepped (i)) return stepValue ((int) std::floor (plain + 0.5), kParams[i].steps);
    return (float) plain;
}

const char* moduleOf (int i)
{
    const int inst = kParams[i].inst;
    return inst < 0 ? "TB Synth" : instrumentName (inst);
}

// Single-producer (main thread) / single-consumer (audio thread) queue.
struct ParamMsg { int16_t index; uint8_t kind; float value; };
enum { kMsgValue = 0, kMsgBegin, kMsgEnd, kMsgValueQuiet };   // quiet: state load, host already knows

class MsgQueue
{
public:
    MsgQueue() : head (0), tail (0) {}
    bool push (const ParamMsg& m)
    {
        const unsigned h = head.load (std::memory_order_relaxed);
        const unsigned n = (h + 1) & (kSize - 1);
        if (n == tail.load (std::memory_order_acquire)) return false;   // full
        buf[h] = m;
        head.store (n, std::memory_order_release);
        return true;
    }
    bool pop (ParamMsg& m)
    {
        const unsigned t = tail.load (std::memory_order_relaxed);
        if (t == head.load (std::memory_order_acquire)) return false;
        m = buf[t];
        tail.store ((t + 1) & (kSize - 1), std::memory_order_release);
        return true;
    }
private:
    enum { kSize = 8192 };     // a preset load sends every parameter three times
    ParamMsg buf[kSize];
    std::atomic<unsigned> head, tail;
};

//==============================================================================
class Plugin : public EditorHost
{
public:
    explicit Plugin (const clap_host_t* h)
        : host (h), hostParams (nullptr), active (false), sampleRate (44100.0),
          editor (nullptr), guiCreated (false)
    {
        std::memset (&plugin, 0, sizeof (plugin));
        plugin.desc = &kDescriptor;
        plugin.plugin_data = this;
        plugin.init = &sInit;
        plugin.destroy = &sDestroy;
        plugin.activate = &sActivate;
        plugin.deactivate = &sDeactivate;
        plugin.start_processing = &sStartProcessing;
        plugin.stop_processing = &sStopProcessing;
        plugin.reset = &sReset;
        plugin.process = &sProcess;
        plugin.get_extension = &sGetExtension;
        plugin.on_main_thread = &sOnMainThread;
        for (int i = 0; i < kNumParams; ++i) values[i].store (kParams[i].def);
        peak.store (0.0f);
    }

    ~Plugin()
    {
#if defined(__APPLE__)
        if (editor) editorClose (editor);
#endif
    }

    clap_plugin_t plugin;

    //--------------------------------------------------------------------------
    // EditorHost (main thread)
    float getParam (int index) const override
    {
        return (index >= 0 && index < kNumParams) ? values[index].load (std::memory_order_relaxed) : 0.0f;
    }
    void getParamText (int index, char* text, int size) const override
    {
        if (index >= 0 && index < kNumParams && size > 0) formatValue (index, getParam (index), text, (size_t) size, true);
        else if (size > 0) text[0] = 0;
    }
    float getMeter (int which) const override { return which == kMeterPeak ? peak.load (std::memory_order_relaxed) : 0.0f; }
    void guiBeginEdit (int index) override { queue (index, kMsgBegin, 0.0f); }
    void guiEndEdit (int index) override   { queue (index, kMsgEnd, 0.0f); requestFlush(); }
    void guiSetParam (int index, float v) override
    {
        if (index < 0 || index >= kNumParams || ! (v == v)) return;
        v = clamp01f (v);
        values[index].store (v);
        queue (index, kMsgValue, v);
        requestFlush();
    }

    // Presets: the same file as the Mountain Lion build, shared by every instance
    void refreshPresets() override { presets.load(); }
    bool presetExists (int slot) const override { return presets.has (slot); }
    bool presetIsFactory (int slot) const override { return PresetStore::isFactory (slot); }
    bool savePreset (int slot) override
    {
        if (PresetStore::isFactory (slot)) return false;
        float v[kNumParams];
        for (int i = 0; i < kNumParams; ++i) v[i] = getParam (i);
        presets.load();
        presets.put (slot, v);
        return presets.save();
    }
    bool loadPreset (int slot) override
    {
        float v[kNumParams];
        if (! presets.get (slot, v)) return false;
        for (int i = 0; i < kNumParams; ++i) { guiBeginEdit (i); guiSetParam (i, v[i]); guiEndEdit (i); }
        return true;
    }
    void presetName (int slot, char* text, int size) const override
    {
        if (! text || size <= 0) return;
        const std::string n = presets.name (slot);
        std::strncpy (text, n.c_str(), (size_t) size - 1);
        text[size - 1] = 0;
    }
    bool renamePreset (int slot, const char* name) override
    {
        presets.load();
        if (! presets.has (slot) || PresetStore::isFactory (slot)) return false;
        presets.setName (slot, name ? name : "");
        return presets.save();
    }
    bool deletePreset (int slot) override
    {
        presets.load();
        if (! presets.has (slot)) return false;
        presets.remove (slot);
        return presets.save();
    }
    bool restoreFactoryPresets (int instrument) override
    {
        presets.load();
        return presets.restoreFactory (instrument) && presets.save();
    }
    bool factoryHidden (int instrument) override
    {
        presets.load();
        return presets.anyHidden (instrument);
    }
    bool copyPreset (int from, int to) override
    {
        presets.load();
        float v[kNumParams];
        if (PresetStore::isFactory (to) || ! presets.get (from, v)) return false;
        const std::string n = presets.name (from);
        presets.put (to, v);
        presets.setName (to, n);
        return presets.save();
    }

private:
    static Plugin* self (const clap_plugin_t* p) { return static_cast<Plugin*> (p->plugin_data); }
    static float clamp01f (float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

    void queue (int index, uint8_t kind, float v)
    {
        ParamMsg m; m.index = (int16_t) index; m.kind = kind; m.value = v;
        if (! msgs.push (m) && kind != kMsgEnd && kind != kMsgBegin) overflowed.store (true);
    }
    void requestFlush()
    {
        if (hostParams && hostParams->request_flush) hostParams->request_flush (host);
    }

    void applyToDsp (int index) { synth.setParam (index, values[index].load (std::memory_order_relaxed)); }

    void drainMessages (const clap_output_events_t* out)
    {
        if (overflowed.exchange (false))
            for (int i = 0; i < kNumParams; ++i) applyToDsp (i);
        ParamMsg m;
        while (msgs.pop (m))
        {
            if (m.index < 0 || m.index >= kNumParams) continue;
            if (m.kind == kMsgValue || m.kind == kMsgValueQuiet) applyToDsp (m.index);
            if (! out || m.kind == kMsgValueQuiet) continue;
            if (m.kind == kMsgValue)
            {
                clap_event_param_value_t e;
                std::memset (&e, 0, sizeof (e));
                e.header.size = sizeof (e);
                e.header.space_id = CLAP_CORE_EVENT_SPACE_ID;
                e.header.type = CLAP_EVENT_PARAM_VALUE;
                e.param_id = (clap_id) m.index;
                e.note_id = -1; e.port_index = -1; e.channel = -1; e.key = -1;
                e.value = toPlain (m.index, m.value);
                out->try_push (out, &e.header);
            }
            else
            {
                clap_event_param_gesture_t e;
                std::memset (&e, 0, sizeof (e));
                e.header.size = sizeof (e);
                e.header.space_id = CLAP_CORE_EVENT_SPACE_ID;
                e.header.type = (m.kind == kMsgBegin) ? CLAP_EVENT_PARAM_GESTURE_BEGIN : CLAP_EVENT_PARAM_GESTURE_END;
                e.param_id = (clap_id) m.index;
                out->try_push (out, &e.header);
            }
        }
    }

    // One input event: a parameter, a note or MIDI
    void handleEvent (const clap_event_header_t* h)
    {
        if (! h || h->space_id != CLAP_CORE_EVENT_SPACE_ID) return;
        switch (h->type)
        {
            case CLAP_EVENT_PARAM_VALUE:
            {
                const clap_event_param_value_t* e = (const clap_event_param_value_t*) h;
                if (e->param_id >= (clap_id) kNumParams) return;
                const int i = (int) e->param_id;
                values[i].store (fromPlain (i, e->value));
                applyToDsp (i);
                break;
            }
            case CLAP_EVENT_NOTE_ON:
            {
                const clap_event_note_t* e = (const clap_event_note_t*) h;
                if (e->key < 0 || e->key > 127) return;
                int vel = (int) std::floor (e->velocity * 127.0 + 0.5);
                if (vel < 1) vel = 1;
                if (vel > 127) vel = 127;
                synth.midi (0x90, (unsigned char) e->key, (unsigned char) vel);
                break;
            }
            case CLAP_EVENT_NOTE_OFF:
            case CLAP_EVENT_NOTE_CHOKE:
            {
                const clap_event_note_t* e = (const clap_event_note_t*) h;
                if (e->key >= 0 && e->key <= 127) synth.midi (0x80, (unsigned char) e->key, 0);
                else if (h->type == CLAP_EVENT_NOTE_CHOKE) synth.allNotesOff();
                break;
            }
            case CLAP_EVENT_MIDI:
            {
                const clap_event_midi_t* e = (const clap_event_midi_t*) h;
                synth.midi (e->data[0], e->data[1], e->data[2]);
                break;
            }
            default: break;
        }
    }

    //--------------------------------------------------------------------------
    // clap_plugin
    static bool sInit (const clap_plugin_t* p)
    {
        Plugin* s = self (p);
        s->hostParams = (const clap_host_params_t*) s->host->get_extension (s->host, CLAP_EXT_PARAMS);
        for (int i = 0; i < kNumParams; ++i) s->applyToDsp (i);
        s->synth.prepare (s->sampleRate);
        return true;
    }
    static void sDestroy (const clap_plugin_t* p) { delete self (p); }
    static bool sActivate (const clap_plugin_t* p, double sr, uint32_t, uint32_t)
    {
        Plugin* s = self (p);
        ParamMsg m;
        while (s->msgs.pop (m)) {}                       // values are already in the atomics
        for (int i = 0; i < kNumParams; ++i) s->applyToDsp (i);
        s->sampleRate = sr;
        s->synth.prepare (sr);
        s->active = true;
        return true;
    }
    static void sDeactivate (const clap_plugin_t* p) { self (p)->active = false; }
    static bool sStartProcessing (const clap_plugin_t*) { return true; }
    static void sStopProcessing (const clap_plugin_t*) {}
    static void sReset (const clap_plugin_t* p) { self (p)->synth.allNotesOff(); }
    static void sOnMainThread (const clap_plugin_t*) {}

    static clap_process_status sProcess (const clap_plugin_t* p, const clap_process_t* proc)
    {
        Plugin* s = self (p);
        s->drainMessages (proc->out_events);

        const uint32_t n = proc->frames_count;
        const clap_input_events_t* in = proc->in_events;
        const uint32_t nev = in ? in->size (in) : 0;
        float* outL = nullptr;
        float* outR = nullptr;
        if (proc->audio_outputs_count >= 1 && proc->audio_outputs[0].data32 && proc->audio_outputs[0].channel_count >= 1)
        {
            outL = proc->audio_outputs[0].data32[0];
            outR = proc->audio_outputs[0].channel_count > 1 ? proc->audio_outputs[0].data32[1] : nullptr;
        }

#if TBSYN_HAVE_SSE
        const unsigned int oldCsr = _mm_getcsr();
        _mm_setcsr (oldCsr | 0x8040);   // flush denormals to zero
#endif
#if defined(__aarch64__)
        uint64_t oldFpcr;
        __asm__ __volatile__ ("mrs %0, fpcr" : "=r" (oldFpcr));
        __asm__ __volatile__ ("msr fpcr, %0" : : "r" (oldFpcr | (1ull << 24)));   // FZ: flush denormals to zero
#endif
        // play each event at its own sample
        uint32_t pos = 0, ev = 0;
        float pk = 0.0f;
        while (pos < n || ev < nev)
        {
            while (ev < nev)
            {
                const clap_event_header_t* h = in->get (in, ev);
                if (h && h->time > pos && h->time < n) break;
                s->handleEvent (h);
                ++ev;
            }
            if (pos >= n) continue;
            uint32_t end = n;
            if (ev < nev)
            {
                const clap_event_header_t* h = in->get (in, ev);
                if (h && h->time < end) end = h->time;
            }
            if (end > pos + (uint32_t) kScratch) end = pos + (uint32_t) kScratch;
            const uint32_t len = end - pos;
            s->synth.process (s->scratch, (int) len);
            for (uint32_t i = 0; i < len; ++i)
            {
                float y = s->scratch[i];
                if (std::fabs (y) < 1.0e-30f) y = 0.0f;
                const float a = std::fabs (y);
                if (a > pk) pk = a;
                if (outL) outL[pos + i] = y;
                if (outR) outR[pos + i] = y;
            }
            pos = end;
        }
        s->peak.store ((float) s->synth.peak(), std::memory_order_relaxed);
        (void) pk;
#if defined(__aarch64__)
        __asm__ __volatile__ ("msr fpcr, %0" : : "r" (oldFpcr));
#endif
#if TBSYN_HAVE_SSE
        _mm_setcsr (oldCsr);
#endif
        return CLAP_PROCESS_CONTINUE;
    }

    //--------------------------------------------------------------------------
    // Extensions
    static const void* sGetExtension (const clap_plugin_t*, const char* id)
    {
        if (! std::strcmp (id, CLAP_EXT_AUDIO_PORTS)) return &audioPortsExt;
        if (! std::strcmp (id, CLAP_EXT_NOTE_PORTS))  return &notePortsExt;
        if (! std::strcmp (id, CLAP_EXT_PARAMS))      return &paramsExt;
        if (! std::strcmp (id, CLAP_EXT_STATE))       return &stateExt;
        if (! std::strcmp (id, CLAP_EXT_TAIL))        return &tailExt;
#if defined(__APPLE__)
        if (! std::strcmp (id, CLAP_EXT_GUI))         return &guiExt;
#endif
        return nullptr;
    }

    // Tail: the longest release, Leslie and chorus ring-out (as the 10.8.5 build)
    static uint32_t tGet (const clap_plugin_t* p) { return (uint32_t) (6.0 * self (p)->sampleRate); }
    static const clap_plugin_tail_t tailExt;

    // Audio ports: one stereo output (the mono signal on both sides), no inputs
    static uint32_t apCount (const clap_plugin_t*, bool isInput) { return isInput ? 0 : 1; }
    static bool apGet (const clap_plugin_t*, uint32_t index, bool isInput, clap_audio_port_info_t* info)
    {
        if (index != 0 || isInput) return false;
        info->id = 0;
        std::snprintf (info->name, sizeof (info->name), "%s", "Out");
        info->flags = CLAP_AUDIO_PORT_IS_MAIN;
        info->channel_count = 2;
        info->port_type = CLAP_PORT_STEREO;
        info->in_place_pair = CLAP_INVALID_ID;
        return true;
    }
    static const clap_plugin_audio_ports_t audioPortsExt;

    // Note ports: one input, CLAP notes or MIDI
    static uint32_t npCount (const clap_plugin_t*, bool isInput) { return isInput ? 1 : 0; }
    static bool npGet (const clap_plugin_t*, uint32_t index, bool isInput, clap_note_port_info_t* info)
    {
        if (index != 0 || ! isInput) return false;
        info->id = 0;
        info->supported_dialects = CLAP_NOTE_DIALECT_CLAP | CLAP_NOTE_DIALECT_MIDI;
        info->preferred_dialect = CLAP_NOTE_DIALECT_CLAP;
        std::snprintf (info->name, sizeof (info->name), "%s", "Notes");
        return true;
    }
    static const clap_plugin_note_ports_t notePortsExt;

    // Parameters
    static uint32_t pCount (const clap_plugin_t*) { return kNumParams; }
    static bool pInfo (const clap_plugin_t*, uint32_t index, clap_param_info_t* info)
    {
        if (index >= (uint32_t) kNumParams) return false;
        const int i = (int) index;
        std::memset (info, 0, sizeof (*info));
        info->id = (clap_id) i;
        info->flags = CLAP_PARAM_IS_AUTOMATABLE | (isStepped (i) ? CLAP_PARAM_IS_STEPPED : 0);
        // names: "Minimoog CUTOFF", so the same word on two instruments stays apart
        if (kParams[i].inst < 0) std::snprintf (info->name, sizeof (info->name), "%s", kParams[i].label);
        else std::snprintf (info->name, sizeof (info->name), "%s %s", instrumentShort (kParams[i].inst), kParams[i].label);
        std::snprintf (info->module, sizeof (info->module), "%s", moduleOf (i));
        info->min_value = 0.0;
        info->max_value = plainMax (i);
        info->default_value = toPlain (i, kParams[i].def);
        return true;
    }
    static bool pValue (const clap_plugin_t* p, clap_id id, double* out)
    {
        if (id >= (clap_id) kNumParams) return false;
        *out = toPlain ((int) id, self (p)->getParam ((int) id));
        return true;
    }
    static bool pToText (const clap_plugin_t*, clap_id id, double value, char* out, uint32_t size)
    {
        if (id >= (clap_id) kNumParams || size == 0) return false;
        formatValue ((int) id, fromPlain ((int) id, value), out, size, true);
        return true;
    }
    static bool pFromText (const clap_plugin_t*, clap_id id, const char* text, double* out)
    {
        if (id >= (clap_id) kNumParams || ! text) return false;
        const int i = (int) id;
        while (*text == ' ') ++text;
        if (! *text) return false;
        // a position by name ("SAW", "ON", "MINIMOOG MODEL D")
        if (isStepped (i))
        {
            char buf[64];
            for (int k = 0; k < plainSteps (i); ++k)
            {
                const float v = i == kInst ? instValue (k) : stepValue (k, kParams[i].steps);
                formatValue (i, v, buf, sizeof (buf), true);
                if (! strcasecmp (buf, text)) { *out = k; return true; }
                formatValue (i, v, buf, sizeof (buf), false);
                if (! strcasecmp (buf, text)) { *out = k; return true; }
            }
            char* e = nullptr;
            const double d = std::strtod (text, &e);
            if (e == text || d < 0.0 || d > plainMax (i)) return false;
            *out = std::floor (d + 0.5);
            return true;
        }
        // a knob: first a position showing exactly this text ("OFF", "-inf", "+0 cents")
        char buf[64];
        for (int k = 0; k <= kKnobSearch; ++k)
        {
            formatValue (i, knobAt (k), buf, sizeof (buf), true);
            if (! strcasecmp (buf, text)) { *out = textRunMiddle (i, text, knobAt (k)); return true; }
        }
        // else the number: find where the knob's reading crosses it, then narrow down
        double unit = 1.0;
        const double want0 = parseNumber (text, &unit);
        if (! (want0 == want0)) return false;
        const bool hasUnit = unit != 0.0;
        const double want = hasUnit ? want0 * unit : want0;
        // (a word such as OFF counts as below every number)
        double prevV = -1.0, prevG = 0.0, bestV = -1.0, bestErr = 1e300, firstCross = -1.0;
        for (int k = 0; k <= kKnobSearch; ++k)
        {
            const double v = knobAt (k);
            double g;
            if (! reading (i, v, hasUnit, g)) g = -1e300;
            else if (std::fabs (g - want) < bestErr) { bestErr = std::fabs (g - want); bestV = v; }
            if (prevV >= 0.0 && (g - want) * (prevG - want) <= 0.0 && g != prevG)
            {
                // bisect between the two grid points
                double lo = prevV, hi = v;
                const bool rising = g > prevG;
                for (int it = 0; it < 40; ++it)
                {
                    const double m = 0.5 * (lo + hi);
                    double gm;
                    if (! reading (i, m, hasUnit, gm)) gm = -1e300;
                    if ((gm < want) == rising) lo = m; else hi = m;
                }
                // the reading equal to the number sits on one side of the crossing
                const double ends[2] = { lo, hi };
                for (int e = 0; e < 2; ++e)
                {
                    formatValue (i, (float) ends[e], buf, sizeof (buf), true);
                    if (! strcasecmp (buf, text)) { *out = textRunMiddle (i, text, ends[e]); return true; }
                }
                // the same number written another way next to it ("1000 Hz" beside "1.00 kHz"): look nearby
                for (int st = 1; st <= 600; ++st)
                    for (int dir = -1; dir <= 1; dir += 2)
                    {
                        const double m = hi + dir * st * 2.0e-6;
                        if (m < 0.0 || m > 1.0) continue;
                        formatValue (i, (float) m, buf, sizeof (buf), true);
                        if (! strcasecmp (buf, text)) { *out = textRunMiddle (i, text, m); return true; }
                    }
                if (firstCross < 0.0) firstCross = 0.5 * (lo + hi);
            }
            prevV = v; prevG = g;
        }
        if (firstCross >= 0.0) { *out = firstCross; return true; }
        if (bestV < 0.0) return false;
        *out = bestV;
        return true;
    }
    static float knobAt (int k) { return (float) k / (float) kKnobSearch; }
    // the knob's reading at v, as a number (in base units when the text gives a unit)
    static bool reading (int i, double v, bool inBaseUnits, double& g)
    {
        char buf[64];
        formatValue (i, (float) v, buf, sizeof (buf), true);
        double u = 0.0;
        g = parseNumber (buf, &u);
        if (! (g == g)) return false;
        if (inBaseUnits && u != 0.0) g *= u;
        return true;
    }
    // the middle of the stretch of travel around v that reads as this same text
    static double textRunMiddle (int i, const char* text, double v)
    {
        char buf[64];
        double edge[2];
        for (int side = 0; side < 2; ++side)
        {
            double in = v, outV = side ? 1.0 : 0.0;
            formatValue (i, (float) outV, buf, sizeof (buf), true);
            if (! strcasecmp (buf, text)) { edge[side] = outV; continue; }
            for (int it = 0; it < 40; ++it)
            {
                const double m = 0.5 * (in + outV);
                formatValue (i, (float) m, buf, sizeof (buf), true);
                if (! strcasecmp (buf, text)) in = m; else outV = m;
            }
            edge[side] = in;
        }
        return 0.5 * (edge[0] + edge[1]);
    }
    enum { kKnobSearch = 1000 };
    // "1.5 kHz" -> 1.5 with *scale 1000; "35 ms" -> 35, 0.001; "20 Hz" / "3 s" -> 1; no unit -> 0. NaN if no number.
    static double parseNumber (const char* t, double* scale)
    {
        char* e = nullptr;
        const double d = std::strtod (t, &e);
        if (e == t) { *scale = 0.0; return std::nan (""); }
        while (*e == ' ') ++e;
        if (! strncasecmp (e, "khz", 3)) *scale = 1000.0;
        else if (! strncasecmp (e, "ms", 2)) *scale = 0.001;
        else if (! strncasecmp (e, "hz", 2) || ((*e == 's' || *e == 'S') && ! std::isalpha ((unsigned char) e[1]))) *scale = 1.0;
        else *scale = 0.0;
        return d;
    }
    static void pFlush (const clap_plugin_t* p, const clap_input_events_t* in, const clap_output_events_t* out)
    {
        Plugin* s = self (p);
        s->drainMessages (out);
        if (! in) return;
        const uint32_t n = in->size (in);
        for (uint32_t k = 0; k < n; ++k)
        {
            const clap_event_header_t* h = in->get (in, k);
            if (h && h->space_id == CLAP_CORE_EVENT_SPACE_ID && h->type == CLAP_EVENT_PARAM_VALUE) s->handleEvent (h);
        }
    }
    static const clap_plugin_params_t paramsExt;

    // State: every parameter as text, so it survives future versions
    static bool sSave (const clap_plugin_t* p, const clap_ostream_t* stream)
    {
        Plugin* s = self (p);
        std::string text = "TBSYNTH-STATE 1\n";
        char line[96];
        for (int i = 0; i < kNumParams; ++i)
        {
            std::snprintf (line, sizeof (line), "%s %.7f\n", kParams[i].name, (double) s->getParam (i));
            text += line;
        }
        const char* data = text.c_str();
        int64_t left = (int64_t) text.size();
        while (left > 0)
        {
            const int64_t w = stream->write (stream, data, (uint64_t) left);
            if (w <= 0) return false;
            data += w; left -= w;
        }
        return true;
    }
    static bool sLoad (const clap_plugin_t* p, const clap_istream_t* stream)
    {
        Plugin* s = self (p);
        std::string text;
        char buf[1024];
        for (int guard = 0; guard < 4096; ++guard)
        {
            const int64_t r = stream->read (stream, buf, sizeof (buf));
            if (r < 0) return false;
            if (r == 0) break;
            text.append (buf, (size_t) r);
        }
        static const char kHeader[] = "TBSYNTH-STATE 1";
        const size_t hl = sizeof (kHeader) - 1;
        if (text.compare (0, hl, kHeader) != 0) return false;
        // lines "<name> <value>": by name, so parameters added later keep their defaults
        size_t at = hl;
        while (at < text.size())
        {
            const size_t eol = text.find ('\n', at);
            const std::string line = text.substr (at, eol == std::string::npos ? std::string::npos : eol - at);
            at = eol == std::string::npos ? text.size() : eol + 1;
            const size_t sp = line.find (' ');
            if (sp == std::string::npos) continue;
            const std::string name = line.substr (0, sp);
            const double v = std::strtod (line.c_str() + sp + 1, nullptr);
            if (! (v == v)) continue;
            for (int i = 0; i < kNumParams; ++i)
            {
                if (name != kParams[i].name) continue;
                const float f = clamp01f ((float) v);
                s->values[i].store (f);
                if (s->active) s->queue (i, kMsgValueQuiet, f);
                else s->applyToDsp (i);
                break;
            }
        }
        if (s->hostParams)
        {
            if (s->hostParams->rescan) s->hostParams->rescan (s->host, CLAP_PARAM_RESCAN_VALUES);
            s->requestFlush();
        }
        return true;
    }
    static const clap_plugin_state_t stateExt;

#if defined(__APPLE__)
    // GUI: the Cocoa panel, fixed size
    static bool gSupported (const clap_plugin_t*, const char* api, bool floating)
    {
        return ! floating && api && ! std::strcmp (api, CLAP_WINDOW_API_COCOA);
    }
    static bool gPreferred (const clap_plugin_t*, const char** api, bool* floating)
    {
        *api = CLAP_WINDOW_API_COCOA; *floating = false; return true;
    }
    static bool gCreate (const clap_plugin_t* p, const char* api, bool floating)
    {
        if (! gSupported (p, api, floating)) return false;
        self (p)->guiCreated = true;
        return true;
    }
    static void gDestroy (const clap_plugin_t* p)
    {
        Plugin* s = self (p);
        if (s->editor) { editorClose (s->editor); s->editor = nullptr; }
        s->guiCreated = false;
    }
    static bool gSetScale (const clap_plugin_t*, double) { return false; }   // Cocoa scales by itself
    static bool gGetSize (const clap_plugin_t*, uint32_t* w, uint32_t* h)
    {
        *w = kEditorWidth; *h = kEditorHeight; return true;
    }
    static bool gCanResize (const clap_plugin_t*) { return false; }
    static bool gHints (const clap_plugin_t*, clap_gui_resize_hints_t*) { return false; }
    static bool gAdjust (const clap_plugin_t*, uint32_t* w, uint32_t* h)
    {
        *w = kEditorWidth; *h = kEditorHeight; return true;
    }
    static bool gSetSize (const clap_plugin_t*, uint32_t w, uint32_t h)
    {
        return w == (uint32_t) kEditorWidth && h == (uint32_t) kEditorHeight;
    }
    static bool gSetParent (const clap_plugin_t* p, const clap_window_t* win)
    {
        Plugin* s = self (p);
        if (! s->guiCreated || ! win || ! win->cocoa) return false;
        if (s->editor) { editorClose (s->editor); s->editor = nullptr; }
        s->editor = editorOpen (win->cocoa, s);
        return s->editor != nullptr;
    }
    static bool gTransient (const clap_plugin_t*, const clap_window_t*) { return false; }
    static void gTitle (const clap_plugin_t*, const char*) {}
    static bool gShow (const clap_plugin_t*) { return true; }
    static bool gHide (const clap_plugin_t*) { return true; }
    static const clap_plugin_gui_t guiExt;
#endif

    //--------------------------------------------------------------------------
    const clap_host_t* host;
    const clap_host_params_t* hostParams;
    std::atomic<float> values[kNumParams];
    std::atomic<float> peak;
    MsgQueue msgs;
    std::atomic<bool> overflowed { false };
    bool active;
    double sampleRate;
    enum { kScratch = 512 };
    Synth synth;
    float scratch[kScratch];
    void* editor;
    bool guiCreated;
    PresetStore presets;
};

const clap_plugin_audio_ports_t Plugin::audioPortsExt = { &Plugin::apCount, &Plugin::apGet };
const clap_plugin_note_ports_t Plugin::notePortsExt = { &Plugin::npCount, &Plugin::npGet };
const clap_plugin_params_t Plugin::paramsExt = { &Plugin::pCount, &Plugin::pInfo, &Plugin::pValue,
                                                 &Plugin::pToText, &Plugin::pFromText, &Plugin::pFlush };
const clap_plugin_tail_t Plugin::tailExt = { &Plugin::tGet };
const clap_plugin_state_t Plugin::stateExt = { &Plugin::sSave, &Plugin::sLoad };
#if defined(__APPLE__)
const clap_plugin_gui_t Plugin::guiExt = { &Plugin::gSupported, &Plugin::gPreferred, &Plugin::gCreate,
                                           &Plugin::gDestroy, &Plugin::gSetScale, &Plugin::gGetSize,
                                           &Plugin::gCanResize, &Plugin::gHints, &Plugin::gAdjust,
                                           &Plugin::gSetSize, &Plugin::gSetParent, &Plugin::gTransient,
                                           &Plugin::gTitle, &Plugin::gShow, &Plugin::gHide };
#endif

//==============================================================================
// Factory

uint32_t fCount (const clap_plugin_factory_t*) { return 1; }
const clap_plugin_descriptor_t* fDescriptor (const clap_plugin_factory_t*, uint32_t index)
{
    return index == 0 ? &kDescriptor : nullptr;
}
const clap_plugin_t* fCreate (const clap_plugin_factory_t*, const clap_host_t* host, const char* id)
{
    if (! host || ! id || std::strcmp (id, kPluginId) != 0) return nullptr;
    if (! clap_version_is_compatible (host->clap_version)) return nullptr;
    Plugin* p = new (std::nothrow) Plugin (host);
    return p ? &p->plugin : nullptr;
}
const clap_plugin_factory_t kClapFactory = { &fCount, &fDescriptor, &fCreate };

} // namespace

bool tbsynth_entry_init (const char*) { return true; }
void tbsynth_entry_deinit() {}
const void* tbsynth_entry_get_factory (const char* factoryId)
{
    if (factoryId && ! std::strcmp (factoryId, CLAP_PLUGIN_FACTORY_ID)) return &kClapFactory;
    return nullptr;
}
