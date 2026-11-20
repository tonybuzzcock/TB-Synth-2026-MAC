// panel_test.mm  (macOS only)
// Loads the built CLAP, opens its panel through the CLAP GUI extension in an
// offscreen window, renders a screenshot of every instrument's page, drives
// every control on every page with synthetic mouse events at the coordinates in
// Layout.h, checks that edits reach the host as gesture / value events, tests
// presets and naming, and closes it again. The checks are the 10.8.5 build's
// editor test, unchanged, through a small shim that looks like its VST host.
//
//   panel_test <path to the .clap binary> <output dir for PNGs>

#import <Cocoa/Cocoa.h>

#include <clap/clap.h>

#include "Editor.h"
#include "Layout.h"
#include "Presets.h"

#include <dlfcn.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace tbsyn;
using namespace tbsyn::layout;

static int failures = 0;
static void check (bool ok, const char* what)
{
    std::printf ("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (! ok) ++failures;
}

// ---- Host ----
static int flushRequests = 0;
static void hpRescan (const clap_host_t*, clap_param_rescan_flags) {}
static void hpClear (const clap_host_t*, clap_id, clap_param_clear_flags) {}
static void hpRequestFlush (const clap_host_t*) { ++flushRequests; }
static const clap_host_params_t hostParams = { hpRescan, hpClear, hpRequestFlush };
static const void* hGetExt (const clap_host_t*, const char* id) { return ! std::strcmp (id, CLAP_EXT_PARAMS) ? &hostParams : nullptr; }
static void hNoop (const clap_host_t*) {}
static const clap_host_t host = { CLAP_VERSION_INIT, nullptr, "panel_test", "TB", "", "1.0", hGetExt, hNoop, hNoop, hNoop };

// What the plugin reports to the host, counted like the VST host callback did
static int automateCalls = 0, beginCalls = 0, endCalls = 0;
static bool sPush (const clap_output_events_t*, const clap_event_header_t* h)
{
    if (h->type == CLAP_EVENT_PARAM_VALUE) ++automateCalls;
    if (h->type == CLAP_EVENT_PARAM_GESTURE_BEGIN) ++beginCalls;
    if (h->type == CLAP_EVENT_PARAM_GESTURE_END) ++endCalls;
    return true;
}
static const clap_output_events_t outEvents = { nullptr, sPush };
static uint32_t noEvents (const clap_input_events_t*) { return 0; }
static const clap_event_header_t* noGet (const clap_input_events_t*, uint32_t) { return nullptr; }
static const clap_input_events_t emptyIn = { nullptr, noEvents, noGet };

// The shim: e->getParameter / e->setParameter with the panel's 0..1 values
struct Fx
{
    const clap_plugin_t* p;
    const clap_plugin_params_t* pp;
    float (*getParameter) (Fx*, int);
    void (*setParameter) (Fx*, int, float);
};
static int plainSteps (int i) { return i == kInst ? kNumInstruments : kParams[i].steps; }
static double toPlain (int i, float v)
{
    if (i == kInst) return instIndex (v);
    if (plainSteps (i) >= 2) return stepOf (v, kParams[i].steps);
    return v;
}
static float fromPlain (int i, double v)
{
    if (i == kInst) return instValue ((int) std::floor (v + 0.5));
    if (plainSteps (i) >= 2) return stepValue ((int) std::floor (v + 0.5), kParams[i].steps);
    return (float) v;
}
static void drain (Fx* e) { e->pp->flush (e->p, &emptyIn, &outEvents); }
static float fxGet (Fx* e, int i)
{
    drain (e);
    double v = 0; e->pp->get_value (e->p, (clap_id) i, &v);
    return fromPlain (i, v);
}
static void fxSet (Fx* e, int i, float value)
{
    clap_event_param_value_t ev; std::memset (&ev, 0, sizeof (ev));
    ev.header.size = sizeof (ev); ev.header.space_id = CLAP_CORE_EVENT_SPACE_ID; ev.header.type = CLAP_EVENT_PARAM_VALUE;
    ev.param_id = (clap_id) i; ev.value = toPlain (i, value); ev.note_id = -1; ev.port_index = -1; ev.channel = -1; ev.key = -1;
    struct L { static uint32_t size (const clap_input_events_t*) { return 1; }
               static const clap_event_header_t* get (const clap_input_events_t* l, uint32_t) { return (const clap_event_header_t*) l->ctx; } };
    clap_input_events_t in = { &ev, L::size, L::get };
    e->pp->flush (e->p, &in, &outEvents);
}
static Fx* gFx = nullptr;

static NSData* renderPNG (NSView* v, const std::string& path)
{
    const NSRect b = [v bounds];
    NSBitmapImageRep* rep = [v bitmapImageRepForCachingDisplayInRect: b];
    [v cacheDisplayInRect: b toBitmapImageRep: rep];
    NSData* png = [rep representationUsingType: NSBitmapImageFileTypePNG properties: [NSDictionary dictionary]];
    [png writeToFile: [NSString stringWithUTF8String: path.c_str()] atomically: YES];
    std::printf ("  wrote %s (%lu bytes)\n", path.c_str(), (unsigned long) [png length]);
    return png;
}

// A mouse event at a point in face coordinates (the face sits inside the frame);
// then whatever the panel queued goes to the "host", as a real host's flush would
static void mouse (NSWindow* w, NSView* v, NSEventType type, NSPoint facePt, int clicks = 1)
{
    const NSPoint wp = [v convertPoint: NSMakePoint (facePt.x + kPanelFrame, facePt.y + kPanelFrame) toView: nil];
    NSEvent* e = [NSEvent mouseEventWithType: type location: wp modifierFlags: 0 timestamp: 0
                                windowNumber: [w windowNumber] context: nil eventNumber: 0 clickCount: clicks pressure: 1.0f];
    if (type == NSEventTypeRightMouseDown) [v rightMouseDown: e];
    else if (type == NSEventTypeLeftMouseDown) [v mouseDown: e];
    else if (type == NSEventTypeLeftMouseDragged) [v mouseDragged: e];
    else [v mouseUp: e];
    if (gFx) drain (gFx);
}
static void click (NSWindow* w, NSView* v, NSPoint p)
{
    mouse (w, v, NSEventTypeLeftMouseDown, p);
    mouse (w, v, NSEventTypeLeftMouseUp, p);
}
static void drag (NSWindow* w, NSView* v, NSPoint p, CGFloat dy)
{
    mouse (w, v, NSEventTypeLeftMouseDown, p);
    mouse (w, v, NSEventTypeLeftMouseDragged, NSMakePoint (p.x, p.y + dy * 0.5));
    mouse (w, v, NSEventTypeLeftMouseDragged, NSMakePoint (p.x, p.y + dy));
    mouse (w, v, NSEventTypeLeftMouseUp, NSMakePoint (p.x, p.y + dy));
}

static void pump (double secs) { [[NSRunLoop currentRunLoop] runUntilDate: [NSDate dateWithTimeIntervalSinceNow: secs]]; }

static NSPoint menuArrowRight() { return NSMakePoint (kMenuX + kMenuW - kArrowW * 0.5, kMenuY + kMenuH * 0.5); }
static NSPoint menuArrowLeft()  { return NSMakePoint (kMenuX + kArrowW * 0.5, kMenuY + kMenuH * 0.5); }

int main (int argc, char** argv)
{
    if (argc < 3) { std::printf ("usage: %s <plugin binary> <png dir>\n", argv[0]); return 2; }
    setvbuf (stdout, 0, _IONBF, 0);
    NSAutoreleasePool* pool = [[NSAutoreleasePool alloc] init];
    [NSApplication sharedApplication];
    const std::string dir = argv[2];
    const std::string presetFile = dir + "/test_presets/Presets.txt";
    setenv ("TBSYNTH_PRESET_FILE", presetFile.c_str(), 1);
    std::remove (presetFile.c_str());

    void* lib = dlopen (argv[1], RTLD_NOW | RTLD_LOCAL);
    if (! lib) { std::printf ("dlopen failed: %s\n", dlerror()); return 1; }
    const clap_plugin_entry_t* entry = (const clap_plugin_entry_t*) dlsym (lib, "clap_entry");
    if (! entry || ! entry->init (argv[1])) { std::printf ("no entry point\n"); return 1; }
    const clap_plugin_factory_t* f = (const clap_plugin_factory_t*) entry->get_factory (CLAP_PLUGIN_FACTORY_ID);
    const clap_plugin_t* plug = f->create_plugin (f, &host, f->get_plugin_descriptor (f, 0)->id);
    if (! plug || ! plug->init (plug)) { std::printf ("create failed\n"); return 1; }
    plug->activate (plug, 44100.0, 1, 8192);
    Fx fx = { plug, (const clap_plugin_params_t*) plug->get_extension (plug, CLAP_EXT_PARAMS), &fxGet, &fxSet };
    Fx* e = &fx;
    gFx = e;

    std::printf ("Editor\n");
    const clap_plugin_gui_t* gui = (const clap_plugin_gui_t*) plug->get_extension (plug, CLAP_EXT_GUI);
    check (gui && gui->is_api_supported (plug, CLAP_WINDOW_API_COCOA, false), "plugin has a Cocoa panel");
    uint32_t gw = 0, gh = 0;
    check (gui->create (plug, CLAP_WINDOW_API_COCOA, false) && gui->get_size (plug, &gw, &gh)
           && gw == (uint32_t) kEditorWidth && gh == (uint32_t) kEditorHeight, "editor size as declared");
    NSWindow* win = [[NSWindow alloc] initWithContentRect: NSMakeRect (0, 0, kEditorWidth, kEditorHeight)
                                                styleMask: NSWindowStyleMaskBorderless backing: NSBackingStoreBuffered defer: NO];
    NSView* parent = [win contentView];
    clap_window_t cw; cw.api = CLAP_WINDOW_API_COCOA; cw.cocoa = parent;
    check (gui->set_parent (plug, &cw) && gui->show (plug), "set_parent opens the panel");
    check ([[parent subviews] count] == 1, "panel view added to the host's view");
    NSView* panel = [[parent subviews] count] ? [[parent subviews] objectAtIndex: 0] : nil;
    if (! panel) return 1;
    check (NSEqualSizes ([panel frame].size, NSMakeSize (kEditorWidth, kEditorHeight)), "panel has the declared size");

    // ------------------------------------------------------------ the INSTRUMENT menu's arrows, every page
    std::printf ("\nInstrument pages\n");
    {
        static const char* const file[kNumInstruments] = { "minimoog", "obx", "solina", "wurlitzer", "hammond", "cembalet", "clavinet", "multiman", "omni", "sk30", "rhapsody", "modular" };
        bool stepped = true;
        std::vector<NSData*> pages;
        for (int k = 0; k < kNumInstruments; ++k)
        {
            const int got = instIndex (e->getParameter (e, kInst));
            if (got != k) { stepped = false; std::printf ("  expected page %d, on %d\n", k, got); }
            pump (0.06);
            NSData* png = renderPNG (panel, dir + "/panel_" + file[k] + ".png");
            pages.push_back (png);
            automateCalls = beginCalls = endCalls = 0;
            click (win, panel, menuArrowRight());
            if (! (automateCalls == 1 && beginCalls == 1 && endCalls == 1)) stepped = false;
        }
        check (stepped, "the right arrow steps through all eight instruments, telling the host each time");
        check (instIndex (e->getParameter (e, kInst)) == 0, "and wraps round to the first");
        click (win, panel, menuArrowLeft());
        check (instIndex (e->getParameter (e, kInst)) == kNumInstruments - 1, "the left arrow steps back (wrapping)");
        bool differ = true;
        for (int a = 0; a < kNumInstruments; ++a)
            for (int b = a + 1; b < kNumInstruments; ++b) if ([pages[a] isEqualToData: pages[b]]) differ = false;
        check (differ, "every instrument has its own page");
        check ([pages[0] length] > 20000, "pages render");
        e->setParameter (e, kInst, 0.0f);
        pump (0.1);
    }

    // ------------------------------------------------------------ every control on every page
    std::printf ("\nEvery control\n");
    {
        bool allOk = true;
        int tested = 0;
        for (int inst = 0; inst < kNumInstruments; ++inst)
        {
            e->setParameter (e, kInst, instValue (inst));
            pump (0.06);
            for (int i = 0; i < kNumCtls; ++i)
            {
                const Ctl& c = kCtls[i];
                const int pinst = kParams[c.param].inst;
                if (pinst != inst && ! (pinst == -1 && inst == 0)) continue;
                // start from a known value: mid for knobs and sliders, a step for switches
                const int n = kParams[c.param].steps;
                float start = n >= 2 ? stepValue (n / 2 - (n == 2 ? 1 : 0), n) : 0.5f;
                if (c.type == kDrawbar) start = stepValue (4, 9);
                std::vector<float> before (kNumParams);
                for (int p = 0; p < kNumParams; ++p) before[p] = e->getParameter (e, p);
                e->setParameter (e, c.param, start);
                before[c.param] = start;
                pump (0.06);
                automateCalls = beginCalls = endCalls = 0;
                const NSPoint at = NSMakePoint (c.x, c.type == kDrawbar ? drawbarY (4) : (c.type == kVSlider ? c.y : c.y));
                switch (c.type)
                {
                    case kKnob:     drag (win, panel, at, -30.0); break;
                    case kVSlider:  drag (win, panel, at, -30.0); break;
                    case kDrawbar:  drag (win, panel, at, kBarStep); break;
                    case kSelector: click (win, panel, at); break;     // a click on the knob steps round
                    case kSlide:    click (win, panel, NSMakePoint (c.x + (n == 2 ? 22.0 : 34.0), c.y)); break;
                    default:        click (win, panel, at); break;
                }
                const float after = e->getParameter (e, c.param);
                bool others = true;
                for (int p = 0; p < kNumParams; ++p) if (p != c.param && e->getParameter (e, p) != before[p]) others = false;
                bool moved;
                if (c.type == kDrawbar) moved = paramStep (c.param, after) == 5;
                else if (n >= 2) moved = paramStep (c.param, after) != paramStep (c.param, start);
                else moved = after > start + 0.05f;
                const bool told = automateCalls > 0 && beginCalls == 1 && endCalls == 1;
                if (! moved || ! others || ! told)
                {
                    allOk = false;
                    std::printf ("  %s (%s): %.3f -> %.3f%s%s\n", kParams[c.param].name, instrumentName (inst), start, after,
                                 others ? "" : ", moved another control", told ? "" : ", host not told");
                }
                e->setParameter (e, c.param, kParams[c.param].def);
                ++tested;
            }
        }
        std::printf ("  %d controls\n", tested);
        check (allOk, "every control on every page moves on its own and tells the host");
        check (tested == kNumParams - 1, "that is every parameter but the instrument itself");
        e->setParameter (e, kInst, 0.0f);
        pump (0.1);       // let the panel's timer turn to the Minimoog page
    }

    // ------------------------------------------------------------ particular controls
    std::printf ("\nControls\n");
    {
        // Minimoog cutoff: drag up, double-click back to default
        const Ctl* cut = 0;
        for (int i = 0; i < kNumCtls; ++i) if (kCtls[i].param == kMgCutoff) cut = &kCtls[i];
        const float c0 = e->getParameter (e, kMgCutoff);
        drag (win, panel, NSMakePoint (cut->x, cut->y), -40.0);
        const float c1 = e->getParameter (e, kMgCutoff);
        check (std::fabs (c1 - (c0 + 0.2f)) < 0.01f, "a knob takes 200 px for its whole turn");
        mouse (win, panel, NSEventTypeLeftMouseDown, NSMakePoint (cut->x, cut->y), 2);
        mouse (win, panel, NSEventTypeLeftMouseUp, NSMakePoint (cut->x, cut->y), 2);
        check (e->getParameter (e, kMgCutoff) == kParams[kMgCutoff].def, "double-click resets a knob");
        // a click on a marked selector position goes straight there
        const Ctl* rng = 0;
        for (int i = 0; i < kNumCtls; ++i) if (kCtls[i].param == kMgO1Rng) rng = &kCtls[i];
        const double a = (-125.0 + 250.0 * 5 / 5.0) * M_PI / 180.0;   // the 2' position
        click (win, panel, NSMakePoint (rng->x + (rng->r + 17.0) * std::sin (a), rng->y - (rng->r + 17.0) * std::cos (a)));
        check (paramStep (kMgO1Rng, e->getParameter (e, kMgO1Rng)) == 5, "clicking the 2' marking turns RANGE to 2'");
        e->setParameter (e, kMgO1Rng, kParams[kMgO1Rng].def);
        // right-click shows the value without changing it
        e->setParameter (e, kMgEmph, 0.3f);
        const Ctl* em = 0;
        for (int i = 0; i < kNumCtls; ++i) if (kCtls[i].param == kMgEmph) em = &kCtls[i];
        mouse (win, panel, NSEventTypeRightMouseDown, NSMakePoint (em->x, em->y));
        check (e->getParameter (e, kMgEmph) == 0.3f, "right-click changes nothing");
        pump (0.1);
        renderPNG (panel, dir + "/panel_rightclick.png");
        e->setParameter (e, kMgEmph, kParams[kMgEmph].def);
    }
    {
        // Hammond: a registration, the Leslie on FAST, screenshot
        e->setParameter (e, kInst, instValue (kHammond));
        const float reg[9] = { 8, 8, 8, 8, 0, 0, 0, 0, 0 };
        for (int b = 0; b < 9; ++b) e->setParameter (e, kB3Db1 + b, reg[b] / 8.0f);
        e->setParameter (e, kLsSpeed, 1.0f);
        e->setParameter (e, kB3PHarm, 0.0f);
        pump (0.1);
        renderPNG (panel, dir + "/panel_hammond_registration.png");
        for (int p = 0; p < kNumParams; ++p) if (kParams[p].inst == kHammond) e->setParameter (e, p, kParams[p].def);
        e->setParameter (e, kInst, 0.0f);
        pump (0.1);
    }

    // ------------------------------------------------------------ presets
    std::printf ("\nPresets\n");
    #define PICK(slot) do { NSMenuItem* it_ = [[[NSMenuItem alloc] initWithTitle: @"" action: nil keyEquivalent: @""] autorelease]; \
                            [it_ setTag: (slot)]; [(id) panel presetPicked: it_]; } while (0)
    const NSPoint savePt = NSMakePoint (kSaveX + kSaveW * 0.5, kSaveY + kSaveH * 0.5);
    const NSPoint defaultPt = NSMakePoint (kDefaultX + kDefaultW * 0.5, kDefaultY + kDefaultH * 0.5);
    {
        // the menus list the factory presets first, numbered in the list only
        e->setParameter (e, kInst, instValue (kOBX));
        pump (0.1);
        pump (0.1);
        NSMenu* m = [(id) panel presetMenu];
        for (int q = 0; q < [m numberOfItems]; ++q) std::printf ("    menu %d: %s\n", q, [[[m itemAtIndex: q] title] UTF8String]);
        check ([m numberOfItems] == 13 && [[[m itemAtIndex: 0] title] isEqualToString: @"1   Queen"]
               && [[[m itemAtIndex: 1] title] isEqualToString: @"2   Japan"] && [[[m itemAtIndex: 2] title] isEqualToString: @"3   Foxx"]
               && [[[m itemAtIndex: 3] title] isEqualToString: @"4   Tom Sawyer"] && [[[m itemAtIndex: 9] title] isEqualToString: @"10   Jarre Brass"],
               "the OB-X menu lists 1 Queen, 2 Japan, 3 Foxx ... 10 Jarre Brass");
        PICK (bankSlot (kOBX, 1));
        check (e->getParameter (e, kOxModPw) == 1.0f && e->getParameter (e, kOxO1Pls) == 1.0f, "choosing 'Japan' from the menu loads it");
        // the arrows step through the menu, wrapping round
        const NSPoint prevPt = NSMakePoint (kPrevX + kStepW * 0.5, kStepY + kStepH * 0.5);
        const NSPoint nextPt = NSMakePoint (kNextX + kStepW * 0.5, kStepY + kStepH * 0.5);
        #define NEAR(p, v) (std::fabs (e->getParameter (e, p) - (v)) < 1e-5f)
        click (win, panel, nextPt);
        check (NEAR (kOxO2Det, 0.64f) && NEAR (kOxPW, 0.25f), "the next arrow loads 3 Foxx after 2 Japan");
        click (win, panel, prevPt);
        check (NEAR (kOxModPw, 1.0f) && NEAR (kOxO2Det, 0.55f), "the previous arrow goes back to 2 Japan");
        click (win, panel, prevPt);
        check (NEAR (kOxO2Det, 0.6f) && NEAR (kOxEnvAm, 0.62f), "and to 1 Queen");
        click (win, panel, prevPt);
        check (NEAR (kOxO2Frq, 0.75f) && NEAR (kOxO2Det, 0.57f), "previous from the first preset wraps to the last (Tenebre)");
        click (win, panel, nextPt);
        check (NEAR (kOxO2Det, 0.6f) && NEAR (kOxEnvAm, 0.62f), "next from the last wraps to the first");
        pump (0.1);
        renderPNG (panel, dir + "/panel_preset_arrows.png");
        // right-click the name: delete it (a factory one is hidden), then restore
        PICK (bankSlot (kOBX, 0));
        NSMenu* acts = [(id) panel presetActions];
        check ([acts numberOfItems] == 4 && [[[acts itemAtIndex: 0] title] isEqualToString: @"Rename \"Queen\"..."]
               && [[[acts itemAtIndex: 1] title] isEqualToString: @"Delete \"Queen\""] && [[acts itemAtIndex: 1] isEnabled]
               && ! [[acts itemAtIndex: 3] isEnabled], "right-click offers Rename and Delete \"Queen\"; nothing to restore yet");
        [(id) panel deleteActivePreset: nil];
        m = [(id) panel presetMenu];
        check ([m numberOfItems] == 12 && [[[m itemAtIndex: 0] title] isEqualToString: @"1   Japan"], "deleting Queen takes it out of the menu, which renumbers");
        acts = [(id) panel presetActions];
        check (! [[acts itemAtIndex: 1] isEnabled] && [[acts itemAtIndex: 3] isEnabled], "then nothing is chosen, and the factory presets can be restored");
        pump (0.1);
        renderPNG (panel, dir + "/panel_preset_deleted.png");
        [(id) panel restoreFactory: nil];
        m = [(id) panel presetMenu];
        check ([m numberOfItems] == 13 && [[[m itemAtIndex: 0] title] isEqualToString: @"1   Queen"], "Restore brings Queen back");
        // Rename: a factory preset becomes your own copy under the new name, at the end
        PICK (bankSlot (kOBX, 1));
        [(id) panel renameActivePreset: nil];
        pump (0.1);
        renderPNG (panel, dir + "/panel_preset_renaming.png");
        [(id) panel renameSlot: bankSlot (kOBX, 1) to: std::string ("Ghosts")];
        m = [(id) panel presetMenu];
        check ([m numberOfItems] == 13 && [[[m itemAtIndex: 0] title] isEqualToString: @"1   Queen"] && [[[m itemAtIndex: 1] title] isEqualToString: @"2   Foxx"]
               && [[[m itemAtIndex: 12] title] isEqualToString: @"13   Ghosts"], "renaming factory 'Japan' to 'Ghosts' moves it to the end as your own");
        check (NEAR (kOxModPw, 1.0f), "and keeps its sound");
        [(id) panel renameSlot: bankSlot (kOBX, 13) to: std::string ("Ghosts 2")];
        m = [(id) panel presetMenu];
        check ([[[m itemAtIndex: 12] title] isEqualToString: @"13   Ghosts 2"], "renaming your own preset keeps its place");
        [(id) panel deleteActivePreset: nil];
        [(id) panel restoreFactory: nil];
        m = [(id) panel presetMenu];
        check ([m numberOfItems] == 13 && [[[m itemAtIndex: 1] title] isEqualToString: @"2   Japan"], "deleting it and restoring puts the menu back as it was");
        PICK (bankSlot (kOBX, 1));
        pump (0.1);
        renderPNG (panel, dir + "/panel_preset_menu.png");
        e->setParameter (e, kInst, instValue (kCembalet));
        pump (0.1);
        m = [(id) panel presetMenu];
        check ([m numberOfItems] == 1 && ! [[m itemAtIndex: 0] isEnabled], "an instrument with no presets says so");
        e->setParameter (e, kInst, instValue (kWurlitzer));
        pump (0.1);
        PICK (bankSlot (kWurlitzer, 0));
        check (std::fabs (e->getParameter (e, kWuVol) - 0.85f) < 1e-5f, "Wurlitzer 'Supertramp'");
    }
    {
        // DEFAULT: this instrument back to its defaults, nothing else
        e->setParameter (e, kWuVib, 0.9f);
        e->setParameter (e, kMgCutoff, 0.9f);
        e->setParameter (e, kMaster, 0.5f);
        pump (0.1);
        automateCalls = 0;
        click (win, panel, defaultPt);
        check (e->getParameter (e, kWuVib) == kParams[kWuVib].def && e->getParameter (e, kWuVol) == kParams[kWuVol].def,
               "DEFAULT puts this instrument's controls back to their defaults");
        check (e->getParameter (e, kMgCutoff) == 0.9f && e->getParameter (e, kMaster) == 0.5f && instIndex (e->getParameter (e, kInst)) == kWurlitzer,
               "and leaves the other instruments, MASTER and the instrument alone");
        check (automateCalls == 2, "telling the host about each control it moved");
        pump (0.1);
        renderPNG (panel, dir + "/panel_default.png");
        e->setParameter (e, kMgCutoff, kParams[kMgCutoff].def);
        e->setParameter (e, kMaster, kParams[kMaster].def);
    }
    {
        // SAVE, name, SAVE: added to the end of this instrument's menu
        e->setParameter (e, kWuVib, 0.8f);
        pump (0.1);
        click (win, panel, savePt);
        NSTextField* field = nil;
        for (NSView* sv in [panel subviews]) if ([sv isKindOfClass: [NSTextField class]]) field = (NSTextField*) sv;
        check (field != nil, "SAVE makes the menu a name field");
        renderPNG (panel, dir + "/panel_naming.png");
        if (field) [field setStringValue: @"Wurli with the vibrato up"];
        click (win, panel, savePt);
        pump (0.2);
        bool named = false;
        char wantLine[96];
        std::snprintf (wantLine, sizeof (wantLine), "name %d: Wurli with the vibrato up", bankSlot (kWurlitzer, 1) + 1);
        FILE* pf = std::fopen (presetFile.c_str(), "r");
        if (pf) { char ln[8192]; while (std::fgets (ln, sizeof (ln), pf)) if (std::strstr (ln, wantLine)) named = true; std::fclose (pf); }
        check (named, "SAVE again adds it to the menu, after 'Supertramp'");
        bool gone = true;
        for (int tries = 0; tries < 10; ++tries)
        {
            gone = true;
            for (NSView* sv in [panel subviews]) if ([sv isKindOfClass: [NSTextField class]]) gone = false;
            if (gone) break;
            pump (0.1);
        }
        check (gone, "and the name field goes away");
        NSMenu* m = [(id) panel presetMenu];
        check ([m numberOfItems] == 2 && [[[m itemAtIndex: 1] title] isEqualToString: @"2   Wurli with the vibrato up"], "the menu shows it as number 2");
        renderPNG (panel, dir + "/panel_preset_saved.png");
        // Return works as the second SAVE too
        e->setParameter (e, kWuVol, 0.4f);
        click (win, panel, savePt);
        field = nil;
        for (NSView* sv in [panel subviews]) if ([sv isKindOfClass: [NSTextField class]]) field = (NSTextField*) sv;
        if (field) { [field setStringValue: @"Quiet one"]; [field sendAction: [field action] to: [field target]]; }
        pump (0.2);
        m = [(id) panel presetMenu];
        check ([m numberOfItems] == 3 && [[[m itemAtIndex: 2] title] isEqualToString: @"3   Quiet one"], "Return adds it too (number 3)");
        // Esc, or a click elsewhere, cancels
        click (win, panel, savePt);
        field = nil;
        for (NSView* sv in [panel subviews]) if ([sv isKindOfClass: [NSTextField class]]) field = (NSTextField*) sv;
        if (field)
        {
            [field setStringValue: @"Not this"];
            [(id) panel control: field textView: nil doCommandBySelector: @selector (cancelOperation:)];
        }
        pump (0.2);
        click (win, panel, savePt);
        click (win, panel, NSMakePoint (300, 300));
        pump (0.2);
        check ([[(id) panel presetMenu] numberOfItems] == 3, "Esc or a click elsewhere cancels: nothing added");
        // recall
        e->setParameter (e, kWuVib, 0.1f);
        e->setParameter (e, kWuVol, 0.9f);
        automateCalls = 0;
        PICK (bankSlot (kWurlitzer, 1));
        check (std::fabs (e->getParameter (e, kWuVib) - 0.8f) < 1e-5f && e->getParameter (e, kWuVol) == kParams[kWuVol].def,
               "choosing it from the menu brings it back");
        check (automateCalls >= kNumParams, "recall tells the host about every parameter");
        renderPNG (panel, dir + "/panel_preset_loaded.png");
        // the same on every instrument: SAVE on the Multiman adds to the Multiman's menu
        e->setParameter (e, kInst, instValue (kMultiman));
        pump (0.1);
        click (win, panel, savePt);
        field = nil;
        for (NSView* sv in [panel subviews]) if ([sv isKindOfClass: [NSTextField class]]) field = (NSTextField*) sv;
        if (field) [field setStringValue: @"Strings"];
        click (win, panel, savePt);
        pump (0.2);
        m = [(id) panel presetMenu];
        check ([m numberOfItems] == 23 && [[[m itemAtIndex: 0] title] isEqualToString: @"1   Full Strings"] && [[[m itemAtIndex: 22] title] isEqualToString: @"23   Strings"], "every instrument has its own menu, working the same way");
    }
    #undef PICK
    for (int i = 0; i < kNumParams; ++i) e->setParameter (e, i, kParams[i].def);

    // ------------------------------------------------------------ host changes show
    std::printf ("\nHost automation shows on the panel\n");
    pump (0.1);
    NSData* pngBefore = renderPNG (panel, dir + "/panel_before_host.png");
    e->setParameter (e, kMgCutoff, 0.9f);
    e->setParameter (e, kMgO2Wave, 0.0f);
    pump (0.3);
    NSData* pngAfter = renderPNG (panel, dir + "/panel_changed.png");
    check (! [pngAfter isEqualToData: pngBefore], "panel redraws after host changes");
    e->setParameter (e, kInst, instValue (kClavinet));
    pump (0.3);
    NSData* pngInst = renderPNG (panel, dir + "/panel_host_instrument.png");
    check (! [pngInst isEqualToData: pngAfter], "the page follows an instrument change from the host");
    for (int i = 0; i < kNumParams; ++i) e->setParameter (e, i, kParams[i].def);

    // ------------------------------------------------------------ the level LED
    std::printf ("\nLevel LED\n");
    {
        pump (0.2);
        NSData* quiet = renderPNG (panel, dir + "/panel_led_off.png");
    // ------------------------------------------------------------ the level LED
    std::printf ("\nLevel LED\n");
    {
        pump (0.2);
        NSData* quiet = renderPNG (panel, dir + "/panel_led_off.png");
        // play a note through the plugin
        plug->start_processing (plug);
        clap_event_midi_t ev; std::memset (&ev, 0, sizeof (ev));
        ev.header.size = sizeof (ev); ev.header.space_id = CLAP_CORE_EVENT_SPACE_ID; ev.header.type = CLAP_EVENT_MIDI;
        ev.data[0] = 0x90; ev.data[1] = 48; ev.data[2] = 110;
        struct L { static uint32_t size (const clap_input_events_t*) { return 1; }
                   static const clap_event_header_t* get (const clap_input_events_t* l, uint32_t) { return (const clap_event_header_t*) l->ctx; } };
        clap_input_events_t in = { &ev, L::size, L::get };
        std::vector<float> Lb (8192), Rb (8192);
        float* outs[2] = { &Lb[0], &Rb[0] };
        clap_audio_buffer_t ob; std::memset (&ob, 0, sizeof (ob)); ob.data32 = outs; ob.channel_count = 2;
        clap_process_t pr; std::memset (&pr, 0, sizeof (pr));
        pr.steady_time = -1; pr.frames_count = 8192; pr.audio_outputs = &ob; pr.audio_outputs_count = 1;
        pr.in_events = &in; pr.out_events = &outEvents;
        plug->process (plug, &pr);
        plug->stop_processing (plug);
        pump (0.2);
        NSData* lit = renderPNG (panel, dir + "/panel_showcase.png");
        check (! [lit isEqualToData: quiet], "the LEVEL LED lights while the instrument plays");
    }

    std::printf ("\nClose\n");
    gui->destroy (plug);
    check ([[parent subviews] count] == 0, "destroying the GUI removes the panel");
    check (gui->create (plug, CLAP_WINDOW_API_COCOA, false) && gui->set_parent (plug, &cw) && [[parent subviews] count] == 1, "reopens");
    gui->destroy (plug);
    gFx = nullptr;
    plug->deactivate (plug);
    plug->destroy (plug);
    entry->deinit();
    [win release];
    [pool release];
    dlclose (lib);

    std::printf ("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PANEL CHECKS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
