/**
 * CHOMPI Endless v0.1
 *
 * Five-layer endless sustain instrument.
 *
 * Controls:
 *   White layer key 1..5 + RED : record/replace that layer
 *   RED while recording        : stop recording
 *   GREEN                      : global Play / Stop
 *   Knob 1                     : WINDOW
 *   Knob 2                     : FADE
 *   Knob 3                     : DRIFT
 *   Knob 4                     : COLOR
 *   Purple encoder             : SPEED / PITCH
 *   RED + Purple encoder       : REVERB
 *   Knob 6                     : MASTER VOLUME
 *
 * Every parameter temporarily takes over the 25 key LEDs as a value bar.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "hardware.h"
#include "temp_led_stuff.h"
#include "daisysp.h"
#include "reverb.h"
#include "EndlessEngine.h"

#define DSY_DTCMRAM_BSS __attribute__((section(".dtcmram_bss")))

using namespace daisy;
using namespace chompi;

namespace
{
constexpr std::size_t kSampleRateHz = 48000;
constexpr float kSampleRate = static_cast<float>(kSampleRateHz);
constexpr std::size_t kMaxLayerSeconds = 4;
constexpr std::size_t kMaxLayerFrames = kSampleRateHz * kMaxLayerSeconds;

constexpr float kMicGain = 5.f;
constexpr float kLineGain = 3.f;
constexpr float kHeadphoneGain = 0.20f;
constexpr float kLineOutGain = 0.30f;

Hardware hw;
neoclo::EndlessEngine engine;
daisysp::Reverb DSY_DTCMRAM_BSS reverb;

int16_t DSY_SDRAM_BSS layer_memory
    [neoclo::EndlessEngine::kLayers]
    [neoclo::EndlessEngine::kBanks]
    [kMaxLayerFrames * 2];

daisysp::DcBlock dc_mic;
daisysp::DcBlock dc_line_l;
daisysp::DcBlock dc_line_r;

// First five WHITE keys: C, D, E, F, G.
constexpr uint8_t kLayerKeyIds[5] = {15, 8, 9, 10, 11};
constexpr uint8_t kLayerLedIds[5] = {24, 23, 22, 21, 20};

// All 25 chromatic key LEDs, left to right.
constexpr uint8_t kPlayableLedIds[25] = {
    24, 0, 23, 1, 22, 21, 2, 20, 3, 19, 4, 18, 17,
    5, 16, 6, 15, 14, 7, 13, 8, 12, 9, 11, 10};

// Official CHOMPI physical -> front-panel logical encoder mapping.
constexpr uint8_t kEncoderMap[6] = {1, 2, 3, 0, 4, 5};

// Front panel knob LEDs for logical knobs 1..6.
constexpr uint8_t kKnobPthLed[6] = {1, 2, 3, 4, 0, 9};

float window_amount = 0.34f;
float fade_amount   = 0.72f;
float drift_amount  = 0.10f;
float color_amount  = 0.12f;
float speed_pos     = 0.50f;
float reverb_amount = 0.18f;
float master_volume = 0.78f;

bool red_held = false;
bool layer_held[5] = {false, false, false, false, false};
std::size_t selected_layer = 0;

uint32_t led_time = 0;
uint32_t battery_time = 0;

enum class ParamDisplay : uint8_t
{
    NONE,
    WINDOW,
    FADE,
    DRIFT,
    COLOR,
    SPEED,
    REVERB,
    MASTER
};

ParamDisplay display_param = ParamDisplay::NONE;
float display_value = 0.f;
uint32_t display_until = 0;

inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

float SpeedFromPosition(float p)
{
    p = Clamp(p, 0.f, 1.f);
    if(p <= 0.5f)
        return 0.25f + p * 1.5f;   // 0.25x .. 1x
    return 1.f + (p - 0.5f) * 2.f; // 1x .. 2x
}

void ZeroSDRAM()
{
    uint32_t* beg = reinterpret_cast<uint32_t*>(0xc0000000);
    const std::size_t words = (1024 * 1024 * 64) / sizeof(uint32_t);
    std::fill(beg, beg + words, 0u);
}

void ShowParam(ParamDisplay param, float value)
{
    display_param = param;
    display_value = Clamp(value, 0.f, 1.f);
    display_until = System::GetNow() + 1300;
}

void ParamColor(ParamDisplay param, float* r, float* g, float* b)
{
    switch(param)
    {
        case ParamDisplay::WINDOW: *r=.04f; *g=.78f; *b=1.f; break;
        case ParamDisplay::FADE:   *r=.55f; *g=.82f; *b=1.f; break;
        case ParamDisplay::DRIFT:  *r=.08f; *g=1.f;  *b=.55f; break;
        case ParamDisplay::COLOR:  *r=1.f;  *g=.34f; *b=.03f; break;
        case ParamDisplay::SPEED:  *r=.72f; *g=.08f; *b=1.f; break;
        case ParamDisplay::REVERB: *r=.20f; *g=.25f; *b=1.f; break;
        case ParamDisplay::MASTER: *r=1.f;  *g=1.f;  *b=1.f; break;
        default:                   *r=0.f;  *g=0.f;  *b=0.f; break;
    }
}

void UpdateReverb()
{
    reverb.SetAmount(reverb_amount * 0.82f);
    reverb.SetInputGain(0.30f + reverb_amount * 0.18f);
    reverb.SetTime(0.68f + reverb_amount * 0.29f);
    reverb.SetDiffusion(0.63f + reverb_amount * 0.28f);
    reverb.SetLowpass(0.38f + (1.f - reverb_amount) * 0.40f);
}

void StartLayerCapture(std::size_t layer)
{
    if(layer >= 5)
        return;

    selected_layer = layer;
    engine.StartCapture(layer);
}

void StopLayerCapture()
{
    if(engine.IsCapturing())
        engine.StopCapture();
}

int HeldLayer()
{
    for(int i = 0; i < 5; ++i)
        if(layer_held[i])
            return i;
    return -1;
}

void ApplyEncoderTurn(uint8_t logical_knob, int inc)
{
    if(inc == 0)
        return;

    switch(logical_knob)
    {
        case 0:
            window_amount = Clamp(window_amount + 0.02f * inc, 0.f, 1.f);
            engine.SetWindow(window_amount);
            ShowParam(ParamDisplay::WINDOW, window_amount);
            break;

        case 1:
            fade_amount = Clamp(fade_amount + 0.02f * inc, 0.f, 1.f);
            engine.SetFade(fade_amount);
            ShowParam(ParamDisplay::FADE, fade_amount);
            break;

        case 2:
            drift_amount = Clamp(drift_amount + 0.02f * inc, 0.f, 1.f);
            engine.SetDrift(drift_amount);
            ShowParam(ParamDisplay::DRIFT, drift_amount);
            break;

        case 3:
            color_amount = Clamp(color_amount + 0.02f * inc, 0.f, 1.f);
            engine.SetColor(color_amount);
            ShowParam(ParamDisplay::COLOR, color_amount);
            break;

        case 4:
            if(red_held)
            {
                reverb_amount = Clamp(reverb_amount + 0.02f * inc, 0.f, 1.f);
                UpdateReverb();
                ShowParam(ParamDisplay::REVERB, reverb_amount);
            }
            else
            {
                speed_pos = Clamp(speed_pos + 0.015f * inc, 0.f, 1.f);
                engine.SetSpeed(SpeedFromPosition(speed_pos));
                ShowParam(ParamDisplay::SPEED, speed_pos);
            }
            break;

        case 5:
            master_volume = Clamp(master_volume + 0.025f * inc, 0.f, 1.f);
            ShowParam(ParamDisplay::MASTER, master_volume);
            break;
    }
}

void ProcessControls()
{
    const int red_id   = static_cast<int>(Hardware::SwId::KEY_26);
    const int green_id = static_cast<int>(Hardware::SwId::KEY_27);

    // Layer keys first so simultaneous WHITE + RED works in the same audio block.
    for(std::size_t i = 0; i < 5; ++i)
    {
        if(hw.button_sr.RisingEdge(kLayerKeyIds[i]))
        {
            layer_held[i] = true;
            selected_layer = i;

            // Also accept RED first, then layer key.
            if(red_held && !engine.IsCapturing())
                StartLayerCapture(i);
        }

        if(hw.button_sr.FallingEdge(kLayerKeyIds[i]))
            layer_held[i] = false;
    }

    // RED is capture/modifier.
    if(hw.button_sr.RisingEdge(red_id))
    {
        red_held = true;

        if(engine.IsCapturing())
        {
            StopLayerCapture();
        }
        else
        {
            const int layer = HeldLayer();
            if(layer >= 0)
                StartLayerCapture(static_cast<std::size_t>(layer));
        }
    }

    if(hw.button_sr.FallingEdge(red_id))
        red_held = false;

    // GREEN = global Play / Stop.
    if(hw.button_sr.RisingEdge(green_id))
    {
        if(engine.IsCapturing())
        {
            StopLayerCapture();
            engine.SetPlaying(true);
        }
        else
        {
            engine.SetPlaying(!engine.IsPlaying());
        }
    }

    for(int physical = 0; physical < 6; ++physical)
    {
        const int inc = hw.enc[physical].Increment();
        if(inc != 0)
            ApplyEncoderTurn(kEncoderMap[physical], inc);
    }
}

void ReadInput(const float* const* in, std::size_t i, float* l, float* r)
{
    if(hw.jack_detect.Read())
    {
        *l = Clamp(dc_line_l.Process(in[2][i] * kLineGain), -1.f, 1.f);
        *r = Clamp(dc_line_r.Process(in[3][i] * kLineGain), -1.f, 1.f);
    }
    else
    {
        const float mic = Clamp(dc_mic.Process(in[0][i] * kMicGain), -1.f, 1.f);
        *l = mic;
        *r = mic;
    }
}

void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   std::size_t size)
{
    hw.ProcessAllControls();
    ProcessControls();

    for(std::size_t i = 0; i < size; ++i)
    {
        float input_l = 0.f;
        float input_r = 0.f;
        ReadInput(in, i, &input_l, &input_r);

        neoclo::EndlessEngine::Stereo sig = engine.Process(input_l, input_r);

        // Monitor the new material while capturing so recording feels immediate.
        if(engine.IsCapturing())
        {
            sig.l += input_l * 0.60f;
            sig.r += input_r * 0.60f;
        }

        float wet_l = sig.l;
        float wet_r = sig.r;
        reverb.Process(&wet_l, &wet_r);

        wet_l *= master_volume;
        wet_r *= master_volume;

        wet_l = Clamp(wet_l, -1.f, 1.f);
        wet_r = Clamp(wet_r, -1.f, 1.f);

        out[0][i] = wet_l * kHeadphoneGain;
        out[1][i] = wet_r * kHeadphoneGain;
        out[2][i] = wet_l * kLineOutGain;
        out[3][i] = wet_r * kLineOutGain;
    }
}

void DrawParameterBar()
{
    float r, g, b;
    ParamColor(display_param, &r, &g, &b);

    const float exact = display_value * 25.f;
    const int full = static_cast<int>(exact);
    const float partial = exact - static_cast<float>(full);

    for(int i = 0; i < 25; ++i)
    {
        float level = 0.018f;

        if(i < full)
            level = 0.85f;
        else if(i == full && i < 25)
            level = 0.18f + partial * 0.67f;

        SetSmtLedFloat(kPlayableLedIds[i],
                       r * level,
                       g * level,
                       b * level);
    }
}

void DrawLayerState()
{
    for(std::size_t i = 0; i < 25; ++i)
        SetSmtLedFloat(kPlayableLedIds[i], 0.f, 0.f, 0.f);

    const int rec_layer = engine.GetCaptureLayer();

    for(std::size_t layer = 0; layer < 5; ++layer)
    {
        if(static_cast<int>(layer) == rec_layer)
        {
            const float pulse = ((System::GetNow() / 120) % 2) ? 1.f : 0.40f;
            SetSmtLedFloat(kLayerLedIds[layer], pulse, 0.02f, 0.01f);
        }
        else if(engine.LayerHasAudio(layer))
        {
            if(layer == selected_layer)
                SetSmtLedFloat(kLayerLedIds[layer], 0.05f, 1.f, 0.72f);
            else
                SetSmtLedFloat(kLayerLedIds[layer], 0.02f, 0.24f, 0.18f);
        }
        else if(layer == selected_layer)
        {
            SetSmtLedFloat(kLayerLedIds[layer], 0.04f, 0.10f, 0.12f);
        }
    }

    // During recording use the remaining LEDs as progress.
    if(engine.IsCapturing())
    {
        const float progress = engine.GetMaxFrames() > 0
            ? static_cast<float>(engine.GetCaptureFrames())
                / static_cast<float>(engine.GetMaxFrames())
            : 0.f;

        const int lit = std::min(25, static_cast<int>(progress * 25.f));
        for(int i = 0; i < lit; ++i)
        {
            // Do not hide the layer-key indicator itself.
            bool is_layer_led = false;
            for(int l = 0; l < 5; ++l)
                if(kPlayableLedIds[i] == kLayerLedIds[l])
                    is_layer_led = true;

            if(!is_layer_led)
                SetSmtLedFloat(kPlayableLedIds[i], 0.32f, 0.01f, 0.01f);
        }
    }
}

void DrawKnobIndicators()
{
    for(int i = 0; i < 10; ++i)
        SetPthLedFloat(i, 0.f, 0.f, 0.f);

    const float vals[6] = {
        window_amount,
        fade_amount,
        drift_amount,
        color_amount,
        speed_pos,
        master_volume
    };

    const ParamDisplay params[6] = {
        ParamDisplay::WINDOW,
        ParamDisplay::FADE,
        ParamDisplay::DRIFT,
        ParamDisplay::COLOR,
        ParamDisplay::SPEED,
        ParamDisplay::MASTER
    };

    for(int i = 0; i < 6; ++i)
    {
        float r, g, b;
        ParamColor(params[i], &r, &g, &b);
        const float bright = 0.05f + vals[i] * 0.60f;
        SetPthLedFloat(kKnobPthLed[i], r * bright, g * bright, b * bright);
    }

    // GREEN transport indicator.
    if(engine.IsPlaying())
        SetPthLedFloat(7, 0.03f, 1.f, 0.18f);
    else
        SetPthLedFloat(7, 0.02f, 0.08f, 0.02f);

    // While RED is held, make the purple/reverb indicator blue-violet.
    if(red_held)
    {
        const float bright = 0.10f + reverb_amount * 0.85f;
        SetPthLedFloat(kKnobPthLed[4],
                       0.18f * bright,
                       0.22f * bright,
                       1.f * bright);
    }
}

void UpdateLeds()
{
    DrawKnobIndicators();

    if(display_param != ParamDisplay::NONE
       && static_cast<int32_t>(display_until - System::GetNow()) > 0)
    {
        DrawParameterBar();
    }
    else
    {
        display_param = ParamDisplay::NONE;
        DrawLayerState();
    }

    fill_led_data();
}

void InitPowerManagement()
{
    hw.MpWrite(0x0c, 0B01010001);
    hw.MpReadAll();

    for(std::size_t i = 0; i < 10; ++i)
    {
        hw.LowBatteryLockoutCheck();
        System::Delay(10);
    }
}

void SettleControlsAndCheckShippingMode()
{
    uint32_t sleep_state = 0;

    for(int i = 0; i < 5000; ++i)
    {
        hw.ProcessAllControls();
        sleep_state += hw.button_sr.State(static_cast<int>(Hardware::SwId::KEY_26))
                    && hw.button_sr.State(static_cast<int>(Hardware::SwId::KEY_27))
                    && hw.button_sr.State(static_cast<int>(Hardware::SwId::KEY_28));
        System::DelayUs(100);
    }

    if(sleep_state > 4000)
        hw.MpWrite(0x08, 0B10111111);
}

void ConfigureUsbPowerHandoff()
{
    hw.usb_sw.Write(false);
    System::Delay(1);
    hw.MpWrite(0x0a, 0B00100100);
    System::Delay(1);
    hw.usb_sw.Write(true);
}

} // namespace

int main(void)
{
    hw.Init();
    InitPowerManagement();

    LedSetup();
    ZeroSDRAM();

    dc_mic.Init(kSampleRate);
    dc_line_l.Init(kSampleRate);
    dc_line_r.Init(kSampleRate);

    int16_t* bank_ptrs[neoclo::EndlessEngine::kLayers]
                      [neoclo::EndlessEngine::kBanks];

    for(std::size_t layer = 0; layer < neoclo::EndlessEngine::kLayers; ++layer)
        for(std::size_t bank = 0; bank < neoclo::EndlessEngine::kBanks; ++bank)
            bank_ptrs[layer][bank] = &layer_memory[layer][bank][0];

    engine.Init(kSampleRate, bank_ptrs, kMaxLayerFrames);
    engine.SetWindow(window_amount);
    engine.SetFade(fade_amount);
    engine.SetDrift(drift_amount);
    engine.SetColor(color_amount);
    engine.SetSpeed(SpeedFromPosition(speed_pos));

    reverb.Init(kSampleRate);
    UpdateReverb();

    SettleControlsAndCheckShippingMode();
    ConfigureUsbPowerHandoff();

    hw.StartAudio(AudioCallback);

    led_time = battery_time = System::GetNow();

    while(1)
    {
        const uint32_t now = System::GetNow();

        if(now - led_time >= 16)
        {
            UpdateLeds();
            led_time = now;
        }

        if(now - battery_time >= 20)
        {
            hw.LowBatteryLockoutCheck();
            battery_time = now;
        }

        System::DelayUs(100);
    }
}
