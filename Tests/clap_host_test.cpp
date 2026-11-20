// clap_host_test.cpp
// A minimal CLAP host: loads the built TB Synth binary, then checks the
// descriptor, ports, parameters, text conversion, notes (CLAP and MIDI, at their
// exact sample), every instrument, automation, state, flush and sample rates.
//
//   clap_host_test <path to the .clap binary>
// On macOS pass the binary inside the bundle:
//   "TB Synth.clap/Contents/MacOS/TB Synth"

#include <clap/clap.h>

#include "Synth.h"

#include <dlfcn.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <string>
#include <vector>

using namespace tbsyn;

static int failures = 0;
static void check (bool ok, const char* what)
{
    std::printf ("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (! ok) ++failures;
}

// ---- Host ----
static int flushRequests = 0, rescans = 0;
static void hpRescan (const clap_host_t*, clap_param_rescan_flags) { ++rescans; }
static void hpClear (const clap_host_t*, clap_id, clap_param_clear_flags) {}
static void hpRequestFlush (const clap_host_t*) { ++flushRequests; }
static const clap_host_params_t hostParams = { hpRescan, hpClear, hpRequestFlush };

static const void* hGetExt (const clap_host_t*, const char* id)
{
    return ! std::strcmp (id, CLAP_EXT_PARAMS) ? &hostParams : nullptr;
}
static void hNoop (const clap_host_t*) {}
static const clap_host_t host = { CLAP_VERSION_INIT, nullptr, "clap_host_test", "TB", "", "1.0",
                                  hGetExt, hNoop, hNoop, hNoop };

// ---- Event lists (kept in time order by the tests) ----
union AnyEvent { clap_event_header_t h; clap_event_param_value_t pv; clap_event_note_t n; clap_event_midi_t m; };

struct InEvents
{
    std::vector<AnyEvent> ev;
    clap_input_events_t list;
    InEvents() { list.ctx = this; list.size = &sSize; list.get = &sGet; }
    static uint32_t sSize (const clap_input_events_t* l) { return (uint32_t) ((InEvents*) l->ctx)->ev.size(); }
    static const clap_event_header_t* sGet (const clap_input_events_t* l, uint32_t i) { return &((InEvents*) l->ctx)->ev[i].h; }
    void param (clap_id id, double v, uint32_t t = 0)
    {
        AnyEvent a; std::memset (&a, 0, sizeof (a));
        clap_event_param_value_t& e = a.pv;
        e.header.size = sizeof (e); e.header.time = t; e.header.space_id = CLAP_CORE_EVENT_SPACE_ID; e.header.type = CLAP_EVENT_PARAM_VALUE;
        e.param_id = id; e.note_id = -1; e.port_index = -1; e.channel = -1; e.key = -1; e.value = v;
        ev.push_back (a);
    }
    void note (bool on, int key, double vel, uint32_t t = 0)
    {
        AnyEvent a; std::memset (&a, 0, sizeof (a));
        clap_event_note_t& e = a.n;
        e.header.size = sizeof (e); e.header.time = t; e.header.space_id = CLAP_CORE_EVENT_SPACE_ID;
        e.header.type = on ? CLAP_EVENT_NOTE_ON : CLAP_EVENT_NOTE_OFF;
        e.note_id = -1; e.port_index = 0; e.channel = 0; e.key = (int16_t) key; e.velocity = vel;
        ev.push_back (a);
    }
    void midi (unsigned char s, unsigned char d1, unsigned char d2, uint32_t t = 0)
    {
        AnyEvent a; std::memset (&a, 0, sizeof (a));
        clap_event_midi_t& e = a.m;
        e.header.size = sizeof (e); e.header.time = t; e.header.space_id = CLAP_CORE_EVENT_SPACE_ID; e.header.type = CLAP_EVENT_MIDI;
        e.port_index = 0; e.data[0] = s; e.data[1] = d1; e.data[2] = d2;
        ev.push_back (a);
    }
};

struct OutEvents
{
    int values = 0, begins = 0, ends = 0;
    clap_output_events_t list;
    OutEvents() { list.ctx = this; list.try_push = &sPush; }
    static bool sPush (const clap_output_events_t* l, const clap_event_header_t* h)
    {
        OutEvents* o = (OutEvents*) l->ctx;
        if (h->type == CLAP_EVENT_PARAM_VALUE) ++o->values;
        if (h->type == CLAP_EVENT_PARAM_GESTURE_BEGIN) ++o->begins;
        if (h->type == CLAP_EVENT_PARAM_GESTURE_END) ++o->ends;
        return true;
    }
};

// ---- Streams ----
struct OStream { std::string data; clap_ostream_t s; OStream() { s.ctx = this; s.write = &w; }
    static int64_t w (const clap_ostream_t* st, const void* b, uint64_t n) { ((OStream*) st->ctx)->data.append ((const char*) b, (size_t) n); return (int64_t) n; } };
struct IStream { std::string data; size_t pos = 0; clap_istream_t s; IStream() { s.ctx = this; s.read = &r; }
    static int64_t r (const clap_istream_t* st, void* b, uint64_t n)
    {
        IStream* is = (IStream*) st->ctx;
        // a few bytes at a time, to prove partial reads work
        size_t k = std::min ((size_t) n, std::min ((size_t) 37, is->data.size() - is->pos));
        std::memcpy (b, is->data.data() + is->pos, k); is->pos += k; return (int64_t) k;
    } };

static bool finite (const std::vector<float>& x, float lim)
{
    for (size_t i = 0; i < x.size(); ++i) if (! (x[i] == x[i]) || std::fabs (x[i]) > lim) return false;
    return true;
}
static double energy (const std::vector<float>& x, size_t from, size_t to)
{
    double s = 0; for (size_t i = from; i < to && i < x.size(); ++i) s += (double) x[i] * x[i]; return s;
}

// Render n samples in blocks; the events go in the first block.
static void run (const clap_plugin_t* p, std::vector<float>& ol, std::vector<float>& orr, uint32_t block,
                 InEvents* in = nullptr, OutEvents* out = nullptr)
{
    InEvents emptyIn; OutEvents sink;
    for (size_t pos = 0; pos < ol.size(); pos += block)
    {
        const uint32_t n = (uint32_t) std::min ((size_t) block, ol.size() - pos);
        float* outs[2] = { &ol[pos], &orr[pos] };
        clap_audio_buffer_t ob; std::memset (&ob, 0, sizeof (ob)); ob.data32 = outs; ob.channel_count = 2;
        clap_process_t pr; std::memset (&pr, 0, sizeof (pr));
        pr.steady_time = -1; pr.frames_count = n;
        pr.audio_inputs = nullptr; pr.audio_inputs_count = 0;
        pr.audio_outputs = &ob; pr.audio_outputs_count = 1;
        pr.in_events = (in && pos == 0) ? &in->list : &emptyIn.list;
        pr.out_events = out ? &out->list : &sink.list;
        p->process (p, &pr);
    }
}

static void allOff (const clap_plugin_t* p, int sr)
{
    InEvents e; e.midi (0xB0, 123, 0);
    std::vector<float> a ((size_t) sr * 7), b ((size_t) sr * 7);
    run (p, a, b, 1024, &e);
}

int main (int argc, char** argv)
{
    if (argc < 2) { std::printf ("usage: %s <plugin binary>\n", argv[0]); return 2; }
    setenv ("TBSYNTH_PRESET_FILE", "/tmp/tbsynth_clap_test_presets.txt", 1);
    if (const char* seed = std::getenv ("TB_SEED")) std::srand ((unsigned) std::atoi (seed));
    std::printf ("Loading %s\n", argv[1]);
    void* lib = dlopen (argv[1], RTLD_NOW | RTLD_LOCAL);
    if (! lib) { std::printf ("dlopen failed: %s\n", dlerror()); return 1; }
    const clap_plugin_entry_t* entry = (const clap_plugin_entry_t*) dlsym (lib, "clap_entry");
    check (entry != nullptr, "exports clap_entry");
    if (! entry) return 1;
    check (entry->init (argv[1]), "entry init");
    const clap_plugin_factory_t* f = (const clap_plugin_factory_t*) entry->get_factory (CLAP_PLUGIN_FACTORY_ID);
    check (f && f->get_plugin_count (f) == 1, "one plugin in the factory");
    const clap_plugin_descriptor_t* d = f->get_plugin_descriptor (f, 0);
    std::printf ("  %s by %s, id %s, version %s\n", d->name, d->vendor, d->id, d->version);
    check (! std::strcmp (d->name, "TB Synth"), "plugin name");
    bool instr = false;
    for (const char* const* ft = d->features; ft && *ft; ++ft) if (! std::strcmp (*ft, CLAP_PLUGIN_FEATURE_INSTRUMENT)) instr = true;
    check (instr, "declared as an instrument");

    const clap_plugin_t* p = f->create_plugin (f, &host, d->id);
    check (p && p->init (p), "create and init");
    if (! p) return 1;

    std::printf ("\nParameters\n");
    const clap_plugin_params_t* pp = (const clap_plugin_params_t*) p->get_extension (p, CLAP_EXT_PARAMS);
    check (pp && pp->count (p) == kNumParams, "every control of all twelve instruments");
    bool defaultsOk = true, roundOk = true, namesUnique = true;
    std::vector<std::string> names;
    for (uint32_t i = 0; i < pp->count (p); ++i)
    {
        clap_param_info_t info;
        pp->get_info (p, i, &info);
        double v = -1; pp->get_value (p, info.id, &v);
        char txt[64] = { 0 }; pp->value_to_text (p, info.id, v, txt, sizeof (txt));
        if (i < 6 || i == kOxCut || i == kB3Db1 || i == kLsSpeed)
            std::printf ("  %-22s %-24s = %-22s (%.3f, %g..%g%s)\n", info.name, info.module, txt, v, info.min_value, info.max_value,
                         (info.flags & CLAP_PARAM_IS_STEPPED) ? ", stepped" : "");
        if (std::fabs (v - info.default_value) > 1e-6) defaultsOk = false;
        for (size_t k = 0; k < names.size(); ++k) if (names[k] == info.name) { namesUnique = false; std::printf ("  duplicate name %s\n", info.name); }
        names.push_back (info.name);
        // every value's text reads back to the same text
        for (int k = 0; k <= 400; ++k)
        {
            double pv = k <= 20 ? info.min_value + (info.max_value - info.min_value) * k / 20.0
                                : info.min_value + (info.max_value - info.min_value) * (double) std::rand() / RAND_MAX;
            if (info.flags & CLAP_PARAM_IS_STEPPED) pv = std::floor (pv + 0.5);
            char t1[64] = { 0 }, t2[64] = { 0 };
            double back = -1;
            pp->value_to_text (p, info.id, pv, t1, sizeof (t1));
            if (! pp->text_to_value (p, info.id, t1, &back)) { roundOk = false; std::printf ("  no parse: %s '%s'\n", info.name, t1); break; }
            pp->value_to_text (p, info.id, back, t2, sizeof (t2));
            if (std::strcmp (t1, t2)) { roundOk = false; std::printf ("  round trip %s: '%s' -> '%s'\n", info.name, t1, t2); break; }
        }
    }
    check (defaultsOk, "values start at the panel's defaults");
    check (namesUnique, "every parameter has its own name");
    check (roundOk, "every parameter's text converts back to the same value");
    {
        clap_param_info_t ii; pp->get_info (p, kInst, &ii);
        check (ii.min_value == 0.0 && ii.max_value == kNumInstruments - 1 && (ii.flags & CLAP_PARAM_IS_STEPPED),
               "INSTRUMENT is stepped, one step per instrument");
        char txt[64]; double tv = -1;
        pp->value_to_text (p, kInst, 2.0, txt, sizeof (txt));
        std::printf ("  instrument 2 reads '%s'\n", txt);
        check (! std::strcmp (txt, instrumentName (kSolina)), "INSTRUMENT reads the instrument's name");
        check (pp->text_to_value (p, kInst, instrumentName (kModular), &tv) && tv == kModular, "and the name converts back");
    }

    std::printf ("\nPorts\n");
    const clap_plugin_audio_ports_t* ap = (const clap_plugin_audio_ports_t*) p->get_extension (p, CLAP_EXT_AUDIO_PORTS);
    clap_audio_port_info_t pi;
    check (ap && ap->count (p, true) == 0 && ap->count (p, false) == 1 && ap->get (p, 0, false, &pi) && pi.channel_count == 2,
           "no audio inputs, one stereo output");
    const clap_plugin_note_ports_t* np = (const clap_plugin_note_ports_t*) p->get_extension (p, CLAP_EXT_NOTE_PORTS);
    clap_note_port_info_t ni;
    check (np && np->count (p, true) == 1 && np->count (p, false) == 0 && np->get (p, 0, true, &ni)
           && (ni.supported_dialects & CLAP_NOTE_DIALECT_MIDI) && (ni.supported_dialects & CLAP_NOTE_DIALECT_CLAP),
           "one note input taking CLAP notes and MIDI");
    const clap_plugin_tail_t* tail = (const clap_plugin_tail_t*) p->get_extension (p, CLAP_EXT_TAIL);

    const int sr = 44100;
    check (p->activate (p, sr, 1, 4096), "activate at 44.1 kHz");
    p->start_processing (p);
    check (tail && tail->get (p) >= (uint32_t) (5 * sr), "reports a tail for releases and the Leslie");

    std::printf ("\nNotes\n");
    {
        // reference: the DSP core directly, the same note
        Synth* ref = new Synth();
        for (int i = 0; i < kNumParams; ++i) ref->setParam (i, kParams[i].def);
        ref->prepare (sr);
        ref->midi (0x90, 57, 100);
        std::vector<float> rl (sr);
        for (int pos = 0; pos < sr; pos += 512) ref->process (&rl[pos], std::min (512, sr - pos));
        delete ref;

        InEvents e; e.note (true, 57, 100.0 / 127.0);
        std::vector<float> ol (sr), orr (sr);
        const std::clock_t t0 = std::clock();
        run (p, ol, orr, 512, &e);
        const double secs = (double) (std::clock() - t0) / CLOCKS_PER_SEC;
        float pk = 0; double diff = 0; bool same = true;
        for (int i = 0; i < sr; ++i)
        {
            pk = std::max (pk, std::fabs (ol[i]));
            diff = std::max (diff, (double) std::fabs (ol[i] - rl[i]));
            if (ol[i] != orr[i]) same = false;
        }
        std::printf ("  peak %.3f, max diff vs DSP core %.2g, CPU %.1f%% of one core\n", pk, diff, 100.0 * secs);
        check (pk > 0.02 && finite (ol, 10.0f), "a CLAP note plays");
        check (diff < 1e-6, "matches the DSP core sample for sample (same as the 10.8.5 build)");
        check (same, "the same signal on both outputs");
        allOff (p, sr);
    }
    {
        // a note at sample 300 of a block: silence before it, sound after
        InEvents e; e.note (true, 60, 0.8, 300);
        std::vector<float> ol (4096), orr (4096);
        run (p, ol, orr, 1024, &e);
        double before = energy (ol, 0, 300), after = energy (ol, 300, 4096);
        std::printf ("  energy before %.3g, after %.3g\n", before, after);
        check (before < 1e-12 && after > 1e-4, "a note starts at its own sample within the block");
        allOff (p, sr);
    }
    {
        InEvents e; e.midi (0x90, 64, 90); e.midi (0xE0, 0, 96, 2000); e.midi (0xB0, 1, 100, 3000);
        std::vector<float> ol (sr / 2), orr (sr / 2);
        run (p, ol, orr, 256, &e);
        check (energy (ol, 0, ol.size()) > 1e-3 && finite (ol, 10.0f), "MIDI notes, pitch bend and mod wheel");
        InEvents off; off.midi (0x80, 64, 0);
        std::vector<float> a ((size_t) sr * 7), b ((size_t) sr * 7);
        run (p, a, b, 512, &off);
        float t = 0; for (size_t i = a.size() - sr; i < a.size(); ++i) t = std::max (t, std::fabs (a[i]));
        std::printf ("  after note off and 6 s: %.2g\n", t);
        check (t < 1e-4f, "note off releases and the sound settles");
    }

    std::printf ("\nEvery instrument\n");
    {
        bool all = true;
        for (int k = 0; k < kNumInstruments; ++k)
        {
            InEvents e; e.param (kInst, k); e.note (true, 48, 0.8, 10); e.note (true, 55, 0.8, 10); e.note (true, 64, 0.8, 10);
            std::vector<float> ol (sr), orr (sr);
            run (p, ol, orr, 512, &e);
            const double en = energy (ol, sr / 4, sr);
            float pk = 0; for (int i = 0; i < sr; ++i) pk = std::max (pk, std::fabs (ol[i]));
            std::printf ("  %-30s peak %.3f\n", instrumentName (k), pk);
            if (! finite (ol, 10.0f) || en < 1e-3) { all = false; std::printf ("    silent or unstable\n"); }
            allOff (p, sr);
        }
        check (all, "all twelve instruments play");
        double v; pp->get_value (p, kInst, &v); check (v == kNumInstruments - 1, "host automation selects the instrument");
    }

    std::printf ("\nAutomation\n");
    {
        bool ok = true;
        for (int k = 0; k < kNumInstruments && ok; ++k)
        {
            InEvents start; start.param (kInst, k); start.note (true, 45, 0.9); start.note (true, 52, 0.9); start.note (true, 69, 0.9);
            std::vector<float> a (64), b (64);
            run (p, a, b, 64, &start);
            for (int blk = 0; blk < 200; ++blk)
            {
                InEvents e;
                const double t = blk / 200.0;
                for (int q = 1; q < kNumParams; ++q)
                {
                    clap_param_info_t info; pp->get_info (p, (uint32_t) q, &info);
                    double v = info.min_value + (info.max_value - info.min_value) * (0.5 + 0.5 * std::sin (6.2831853 * (q + 1) * t));
                    if (info.flags & CLAP_PARAM_IS_STEPPED) v = std::floor (v + 0.5);
                    e.param ((clap_id) q, v);
                }
                std::vector<float> o1 (64), o2 (64);
                run (p, o1, o2, 64, &e);
                if (! finite (o1, 100.0f)) { ok = false; std::printf ("  %s unstable\n", instrumentName (k)); break; }
            }
            allOff (p, sr);
        }
        check (ok, "every control automated together while playing, on every instrument");
    }

    std::printf ("\nState\n");
    {
        const clap_plugin_state_t* st = (const clap_plugin_state_t*) p->get_extension (p, CLAP_EXT_STATE);
        InEvents set; set.param (kInst, kHammond); set.param (kB3Db3, 0.25); set.param (kLsSpeed, 1.0); set.param (kOxCut, 0.3);
        std::vector<float> a0 (512), b0 (512);
        run (p, a0, b0, 512, &set);
        std::vector<double> before (kNumParams);
        for (int i = 0; i < kNumParams; ++i) pp->get_value (p, (clap_id) i, &before[i]);
        OStream os;
        check (st && st->save (p, &os.s) && os.data.size() > 200, "saves state");
        InEvents in; in.param (kInst, kSolina); in.param (kB3Db3, 7.0); in.param (kLsSpeed, 0.0); in.param (kOxCut, 0.8);
        std::vector<float> a (512), b (512);
        run (p, a, b, 512, &in);
        IStream is; is.data = os.data;
        rescans = 0;
        check (st->load (p, &is.s), "loads state (in small pieces)");
        bool same = true;
        for (int i = 0; i < kNumParams; ++i)
        {
            double v; pp->get_value (p, (clap_id) i, &v);
            if (std::fabs (v - before[i]) > 1e-5) { same = false; std::printf ("  %s: %g, was %g\n", kParams[i].name, v, before[i]); }
        }
        check (same && before[kInst] == kHammond, "state restores the instrument and every control");
        check (rescans > 0, "host is told to re-read the values");
        IStream junk; junk.data = "not a state";
        check (! st->load (p, &junk.s), "rejects foreign data");
    }

    std::printf ("\nFlush while stopped\n");
    {
        p->stop_processing (p);
        InEvents in; in.param (kMaster, 0.25);
        OutEvents out;
        pp->flush (p, &in.list, &out.list);
        double v; pp->get_value (p, kMaster, &v);
        check (std::fabs (v - 0.25) < 1e-6, "flush applies host changes without audio running");
        p->start_processing (p);
    }

    std::printf ("\nSample rates\n");
    const double rates[] = { 1200.0, 22050.0, 96000.0, 192000.0 };
    for (int ri = 0; ri < 4; ++ri)
    {
        const double r = rates[ri];
        p->stop_processing (p);
        p->deactivate (p);
        bool ok = p->activate (p, r, 1, 4096);
        p->start_processing (p);
        for (int k = 0; k < kNumInstruments && ok; ++k)
        {
            InEvents e; e.param (kInst, k); e.note (true, 36, 0.9); e.note (true, 84, 0.9); e.note (true, 108, 0.9);
            std::vector<float> a ((size_t) r / 2), b ((size_t) r / 2);
            run (p, a, b, 1024, &e);
            if (! finite (a, 10.0f)) { ok = false; std::printf ("  %s unstable at %g Hz\n", instrumentName (k), r); }
            InEvents off; off.midi (0xB0, 123, 0);
            std::vector<float> c (1024), dd (1024);
            run (p, c, dd, 1024, &off);
        }
        char what[64]; std::snprintf (what, sizeof (what), "every instrument runs at %g Hz", r);
        check (ok, what);
    }

    p->stop_processing (p);
    p->deactivate (p);
    p->destroy (p);
    entry->deinit();
    check (true, "destroy and deinit");
    dlclose (lib);

    std::printf ("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL CLAP HOST CHECKS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
