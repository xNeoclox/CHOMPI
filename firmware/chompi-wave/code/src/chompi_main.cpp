/**
 * CHOMPI Decay Memory v0.3
 *
 * Controls:
 *   Red CHOMPI button : record. Hold/release OR tap/tap to stop.
 *   Green PLAY button : playback stop/start.
 *   LOOP button       : Decay HOLD.
 *   Knob 1            : slice size.
 *   Knob 2            : mutation.
 *   Knob 3            : AGE.
 *   Knob 4            : glitch / bitcrush.
 *   Purple knob       : global varispeed (0.25x .. 2x, center = 1x).
 *   Knob 6            : tape delay / space.
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
constexpr std::size_t kDelayFrames = kSampleRateHz * 2;

constexpr float kMicGain = 5.f;
constexpr float kLineGain = 3.f;
constexpr float kHeadphoneGain = 0.20f;
constexpr float kLineOutGain = 0.30f;

Hardware hw;
neoclo::DecayMemoryEngine memory_engine;

int16_t DSY_SDRAM_BSS decay_memory[kMaxCaptureFrames * 2];
float DSY_SDRAM_BSS delay_memory[kDelayFrames * 2];

daisysp::DcBlock dc_mic;
daisysp::DcBlock dc_line_l;
daisysp::DcBlock dc_line_r;

constexpr uint8_t kPlayableKeyIds[25] = {
    15, 7, 8, 12, 9, 10, 13, 11, 14, 16, 21, 17, 18,
    22, 19, 23, 20, 24, 29, 25, 30, 26, 31, 27, 28};

constexpr uint8_t kPlayableLedIds[25] = {
    24, 0, 23, 1, 22, 21, 2, 20, 3, 19, 4, 18, 17,
    5, 16, 6, 15, 14, 7, 13, 8, 12, 9, 11, 10};

// Same front-panel remap used by the official CHOMPI UI.
constexpr uint8_t kEncoderMap[6] = {1, 2, 3, 0, 4, 5};

float slice_size = 0.32f;
float mutation   = 0.25f;
float age        = 0.10f;
float glitch     = 0.f;
float speed_pos  = 0.50f;
float space      = 0.f;

uint32_t led_time = 0;
uint32_t battery_time = 0;
uint32_t record_press_time = 0;
bool record_started_this_press = false;

inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

inline float InputClip(float x)
{
    return Clamp(x, -1.f, 1.f);
}

float SpeedFromPosition(float p)
{
    p = Clamp(p, 0.f, 1.f);

    // Center detent is musically useful: exactly 1x.
    if(p <= 0.5f)
        return 0.25f + p * 1.5f;      // 0.25x .. 1x
    return 1.f + (p - 0.5f) * 2.f;    // 1x .. 2x
}

class TapeSpace
{
  public:
    void Init(float* memory, std::size_t max_frames)
    {
        mem_ = memory;
        max_frames_ = max_frames;
        write_ = 0;
        amount_ = 0.f;
        feedback_state_l_ = 0.f;
        feedback_state_r_ = 0.f;
    }

    void SetAmount(float amount)
    {
        amount_ = Clamp(amount, 0.f, 1.f);
    }

    neoclo::DecayMemoryEngine::Stereo Process(float in_l, float in_r)
    {
        if(!mem_ || max_frames_ < 2)
            return {in_l, in_r};

        const std::size_t min_delay = static_cast<std::size_t>(kSampleRate * 0.12f);
        const std::size_t max_delay = static_cast<std::size_t>(kSampleRate * 0.95f);
        const std::size_t delay = min_delay
            + static_cast<std::size_t>(amount_ * amount_ * static_cast<float>(max_delay - min_delay));

        const std::size_t read = (write_ + max_frames_ - std::min(delay, max_frames_ - 1)) % max_frames_;

        const float wet_l = mem_[read * 2];
        const float wet_r = mem_[read * 2 + 1];

        const float lp = 0.06f + (1.f - amount_) * 0.12f;
        feedback_state_l_ += lp * (wet_l - feedback_state_l_);
        feedback_state_r_ += lp * (wet_r - feedback_state_r_);

        const float feedback = 0.18f + amount_ * 0.68f;

        // Cross feedback gives a subtle ping-pong movement.
        mem_[write_ * 2] = Clamp(in_l + feedback_state_r_ * feedback, -1.f, 1.f);
        mem_[write_ * 2 + 1] = Clamp(in_r + feedback_state_l_ * feedback, -1.f, 1.f);

        write_++;
        if(write_ >= max_frames_)
            write_ = 0;

        const float mix = amount_ * 0.55f;
        return {
            in_l + (wet_l - in_l) * mix,
            in_r + (wet_r - in_r) * mix
        };
    }

  private:
    float* mem_ = nullptr;
    std::size_t max_frames_ = 0;
    std::size_t write_ = 0;
    float amount_ = 0.f;
    float feedback_state_l_ = 0.f;
    float feedback_state_r_ = 0.f;
};

TapeSpace tape_space;

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
        case 0: // slice size
            slice_size = Clamp(slice_size + 0.025f * inc, 0.f, 1.f);
            memory_engine.SetSliceSize(slice_size);
            break;

        case 1: // mutation
            mutation = Clamp(mutation + 0.02f * inc, 0.f, 1.f);
            memory_engine.SetMutation(mutation);
            break;

        case 2: // AGE
            age = Clamp(age + 0.02f * inc, 0.f, 1.f);
            memory_engine.SetAge(age);
            break;

        case 3: // glitch / bitcrush
            glitch = Clamp(glitch + 0.02f * inc, 0.f, 1.f);
            memory_engine.SetGlitch(glitch);
            break;

        case 4: // big purple knob: global varispeed
            speed_pos = Clamp(speed_pos + 0.015f * inc, 0.f, 1.f);
            memory_engine.SetPlaybackSpeed(SpeedFromPosition(speed_pos));
            break;

        case 5: // tape delay / space
            space = Clamp(space + 0.02f * inc, 0.f, 1.f);
            tape_space.SetAmount(space);
            break;
    }
}

void StartRecording()
{
    memory_engine.StartCapture();
    record_press_time = System::GetNow();
    record_started_this_press = true;
}

void StopRecording()
{
    memory_engine.StopCapture();
    record_started_this_press = false;
}

void ProcessControls()
{
    const int rec_id = static_cast<int>(Hardware::SwId::KEY_26);
    const int play_id = static_cast<int>(Hardware::SwId::KEY_27);
    const int hold_id = static_cast<int>(Hardware::SwId::KEY_28);

    // RED CHOMPI button.
    // Supports both workflows:
    //   hold > 250 ms and release = stop
    //   quick tap to latch, second tap = stop
    if(hw.button_sr.RisingEdge(rec_id))
    {
        if(memory_engine.IsCapturing())
            StopRecording();
        else
            StartRecording();
    }

    if(hw.button_sr.FallingEdge(rec_id))
    {
        const uint32_t held_ms = System::GetNow() - record_press_time;
        if(memory_engine.IsCapturing() && record_started_this_press && held_ms >= 250)
            StopRecording();

        record_started_this_press = false;
    }

    // GREEN button = playback STOP / PLAY.
    if(hw.button_sr.RisingEdge(play_id))
    {
        if(memory_engine.IsCapturing())
        {
            StopRecording();
            memory_engine.SetPlaying(true);
        }
        else
        {
            memory_engine.SetPlaying(!memory_engine.IsPlaying());
        }
    }

    // LOOP button keeps the current generation from aging further.
    if(hw.button_sr.RisingEdge(hold_id))
        memory_engine.SetDecayHold(!memory_engine.GetDecayHold());

    if(!memory_engine.IsCapturing() && memory_engine.HasAudio())
    {
        for(std::size_t key = 0; key < 25; ++key)
        {
            if(hw.button_sr.RisingEdge(kPlayableKeyIds[key]))
                memory_engine.TriggerSlice(key, false, 1.f);
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

        neoclo::DecayMemoryEngine::Stereo sig = memory_engine.Process(input_l, input_r);

        if(!memory_engine.IsCapturing())
            sig = tape_space.Process(sig.l, sig.r);

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
            ? static_cast<float>(memory_engine.GetRecordedFrames())
                / static_cast<float>(memory_engine.GetMaxFrames())
            : 0.f;

        const std::size_t lit = std::min<std::size_t>(
            25, static_cast<std::size_t>(p * 25.f) + 1);

        for(std::size_t i = 0; i < lit; ++i)
            SetSmtLedFloat(kPlayableLedIds[i], 1.f, 0.05f, 0.02f);

        SetPthLedFloat(0, 1.f, 0.04f, 0.02f);
    }
    else if(memory_engine.HasAudio())
    {
        for(std::size_t i = 0; i < 25; ++i)
            SetSmtLedFloat(kPlayableLedIds[i], 0.015f, 0.07f, 0.07f);

        const std::size_t current = std::min<std::size_t>(
            24, memory_engine.GetCurrentSlice());
        SetSmtLedFloat(kPlayableLedIds[current], 0.10f, 1.f, 0.82f);
    }

    // Green PLAY light.
    if(memory_engine.IsPlaying())
        SetPthLedFloat(7, 0.05f, 1.f, 0.25f);

    // HOLD light.
    if(memory_engine.GetDecayHold())
        SetPthLedFloat(8, 1.f, 0.55f, 0.03f);

    // AGE amount indicator.
    SetPthLedFloat(3, age, age * 0.06f, 0.f);

    // Glitch amount indicator.
    SetPthLedFloat(4, glitch * 0.65f, 0.f, glitch);

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
    memory_engine.SetSliceSize(slice_size);
    memory_engine.SetMutation(mutation);
    memory_engine.SetAge(age);
    memory_engine.SetCharacter(0.58f);
    memory_engine.SetMemory(0.72f);
    memory_engine.SetGlitch(glitch);
    memory_engine.SetPlaybackSpeed(SpeedFromPosition(speed_pos));
    memory_engine.SetOutputGain(0.90f);

    tape_space.Init(delay_memory, kDelayFrames);
    tape_space.SetAmount(space);

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
