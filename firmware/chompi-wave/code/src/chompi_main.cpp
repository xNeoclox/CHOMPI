/**
 * CHOMPI Decay Memory v0.2
 *
 * Standalone custom firmware built on CHOMPI WAVE's hardware layer.
 * Main concept: capture -> slices -> probabilistic playback -> destructive aging -> write back.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "hardware.h"
#include "temp_led_stuff.h"
#include "daisysp.h"
#include "DecayMemoryEngine.h"

using namespace daisy;
using namespace chompi;

namespace
{
constexpr std::size_t kSampleRateHz = 48000;
constexpr float kSampleRate = static_cast<float>(kSampleRateHz);
constexpr std::size_t kMaxCaptureSeconds = 30;
constexpr std::size_t kMaxCaptureFrames = kSampleRateHz * kMaxCaptureSeconds;

constexpr float kMicGain = 5.f;
constexpr float kLineGain = 3.f;
constexpr float kHeadphoneGain = 0.20f;
constexpr float kLineOutGain = 0.30f;

Hardware hw;
neoclo::DecayMemoryEngine memory_engine;

int16_t DSY_SDRAM_BSS decay_memory[kMaxCaptureFrames * 2];

daisysp::DcBlock dc_mic;
daisysp::DcBlock dc_line_l;
daisysp::DcBlock dc_line_r;

constexpr uint8_t kPlayableKeyIds[25] = {
    15, 7, 8, 12, 9, 10, 13, 11, 14, 16, 21, 17, 18,
    22, 19, 23, 20, 24, 29, 25, 30, 26, 31, 27, 28};

constexpr uint8_t kPlayableLedIds[25] = {
    24, 0, 23, 1, 22, 21, 2, 20, 3, 19, 4, 18, 17,
    5, 16, 6, 15, 14, 7, 13, 8, 12, 9, 11, 10};

constexpr uint8_t kEncoderMap[6] = {1, 2, 3, 0, 4, 5};

std::size_t slice_count = 25;
float mutation  = 0.25f;
float age       = 0.20f;
float character = 0.35f;
float memory    = 0.45f;
float output    = 0.90f;

uint32_t led_time = 0;
uint32_t battery_time = 0;

inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

inline float InputClip(float x)
{
    return Clamp(x, -1.f, 1.f);
}

void ZeroSDRAM()
{
    uint32_t* beg = reinterpret_cast<uint32_t*>(0xc0000000);
    const std::size_t words = (1024 * 1024 * 64) / sizeof(uint32_t);
    std::fill(beg, beg + words, 0u);
}

void ApplyEncoderTurn(uint8_t logical_knob, int inc)
{
    if(inc == 0)
        return;

    switch(logical_knob)
    {
        case 0:
        {
            int next = static_cast<int>(slice_count) + inc;
            if(next < 4) next = 4;
            if(next > 25) next = 25;
            slice_count = static_cast<std::size_t>(next);
            memory_engine.SetSliceCount(slice_count);
            break;
        }
        case 1:
            mutation = Clamp(mutation + 0.02f * inc, 0.f, 1.f);
            memory_engine.SetMutation(mutation);
            break;
        case 2:
            age = Clamp(age + 0.02f * inc, 0.f, 1.f);
            memory_engine.SetAge(age);
            break;
        case 3:
            character = Clamp(character + 0.02f * inc, 0.f, 1.f);
            memory_engine.SetCharacter(character);
            break;
        case 4:
            memory = Clamp(memory + 0.02f * inc, 0.f, 1.f);
            memory_engine.SetMemory(memory);
            break;
        case 5:
            output = Clamp(output + 0.03f * inc, 0.f, 1.5f);
            memory_engine.SetOutputGain(output);
            break;
    }
}

void ProcessControls()
{
    if(hw.button_sr.RisingEdge(static_cast<int>(Hardware::SwId::KEY_26)))
    {
        if(memory_engine.IsCapturing())
            memory_engine.StopCapture();
        else
            memory_engine.StartCapture();
    }

    if(hw.button_sr.RisingEdge(static_cast<int>(Hardware::SwId::KEY_27)))
        memory_engine.SetPlaying(!memory_engine.IsPlaying());

    if(hw.button_sr.RisingEdge(static_cast<int>(Hardware::SwId::KEY_28)))
        memory_engine.SetDecayHold(!memory_engine.GetDecayHold());

    if(!memory_engine.IsCapturing() && memory_engine.HasAudio())
    {
        for(std::size_t key = 0; key < 25; ++key)
        {
            if(hw.button_sr.RisingEdge(kPlayableKeyIds[key]))
            {
                const std::size_t target = (key * memory_engine.GetSliceCount()) / 25;
                memory_engine.TriggerSlice(target, false, 1.f);
            }
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
        *l = InputClip(dc_line_l.Process(in[2][i] * kLineGain));
        *r = InputClip(dc_line_r.Process(in[3][i] * kLineGain));
    }
    else
    {
        const float mic = InputClip(dc_mic.Process(in[0][i] * kMicGain));
        *l = mic;
        *r = mic;
    }
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, std::size_t size)
{
    hw.ProcessAllControls();
    ProcessControls();

    for(std::size_t i = 0; i < size; ++i)
    {
        float input_l = 0.f;
        float input_r = 0.f;
        ReadInput(in, i, &input_l, &input_r);

        const neoclo::DecayMemoryEngine::Stereo sig = memory_engine.Process(input_l, input_r);

        out[0][i] = sig.l * kHeadphoneGain;
        out[1][i] = sig.r * kHeadphoneGain;
        out[2][i] = sig.l * kLineOutGain;
        out[3][i] = sig.r * kLineOutGain;
    }
}

void UpdateLeds()
{
    for(std::size_t i = 0; i < 25; ++i)
        SetSmtLedFloat(kPlayableLedIds[i], 0.f, 0.f, 0.f);

    for(std::size_t i = 0; i < 10; ++i)
        SetPthLedFloat(i, 0.f, 0.f, 0.f);

    if(memory_engine.IsCapturing())
    {
        const float p = memory_engine.GetMaxFrames() > 0
                            ? static_cast<float>(memory_engine.GetRecordedFrames()) /
                                  static_cast<float>(memory_engine.GetMaxFrames())
                            : 0.f;
        const std::size_t lit = std::min<std::size_t>(25, static_cast<std::size_t>(p * 25.f) + 1);
        for(std::size_t i = 0; i < lit; ++i)
            SetSmtLedFloat(kPlayableLedIds[i], 1.f, 0.05f, 0.02f);

        SetPthLedFloat(0, 1.f, 0.04f, 0.02f);
    }
    else if(memory_engine.HasAudio())
    {
        for(std::size_t i = 0; i < 25; ++i)
            SetSmtLedFloat(kPlayableLedIds[i], 0.015f, 0.07f, 0.07f);

        const std::size_t sc = std::max<std::size_t>(1, memory_engine.GetSliceCount());
        const std::size_t current = std::min<std::size_t>(24,
            (memory_engine.GetCurrentSlice() * 25) / sc);
        SetSmtLedFloat(kPlayableLedIds[current], 0.10f, 1.f, 0.82f);

        SetPthLedFloat(0, 0.05f, 0.45f, 0.42f);
    }

    if(memory_engine.IsPlaying())
        SetPthLedFloat(7, 0.05f, 1.f, 0.25f);
    if(memory_engine.GetDecayHold())
        SetPthLedFloat(8, 1.f, 0.55f, 0.03f);

    SetPthLedFloat(3, age, age * 0.12f, 0.f);

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

    memory_engine.Init(kSampleRate, decay_memory, kMaxCaptureFrames);
    memory_engine.SetSliceCount(slice_count);
    memory_engine.SetMutation(mutation);
    memory_engine.SetAge(age);
    memory_engine.SetCharacter(character);
    memory_engine.SetMemory(memory);
    memory_engine.SetOutputGain(output);

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
