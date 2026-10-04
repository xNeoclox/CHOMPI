#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace neoclo
{

class EndlessEngine
{
  public:
    static constexpr std::size_t kLayers = 5;
    static constexpr std::size_t kBanks  = 2;

    struct Stereo
    {
        float l;
        float r;
    };

    void Init(float sample_rate,
              int16_t* bank_memory[kLayers][kBanks],
              std::size_t max_frames)
    {
        sample_rate_ = sample_rate > 1000.f ? sample_rate : 48000.f;
        max_frames_  = max_frames;

        for(std::size_t l = 0; l < kLayers; ++l)
        {
            for(std::size_t b = 0; b < kBanks; ++b)
            {
                mem_[l][b] = bank_memory[l][b];
                frames_[l][b] = 0;
                phase_[l][b] = 0.f;
                drift_start_[l][b] = 0.f;
            }

            active_bank_[l] = 0;
            crossfade_bank_[l] = -1;
            bank_xfade_[l] = 0.f;
            bank_xfade_inc_[l] = 0.f;
        }

        capture_layer_ = -1;
        capture_bank_  = -1;
        capture_write_ = 0;

        playing_ = true;

        window_ = 0.34f;
        fade_   = 0.72f;
        drift_  = 0.10f;
        color_  = 0.12f;
        speed_  = 1.f;
        master_ = 0.85f;

        lp_l_ = lp_r_ = 0.f;
        UpdateColor();
    }

    void StartCapture(std::size_t layer)
    {
        if(layer >= kLayers)
            return;

        capture_layer_ = static_cast<int>(layer);
        capture_bank_  = 1 - active_bank_[layer];
        capture_write_ = 0;
        frames_[layer][capture_bank_] = 0;
        phase_[layer][capture_bank_] = 0.f;
        drift_start_[layer][capture_bank_] = 0.f;
    }

    void StopCapture()
    {
        if(capture_layer_ < 0 || capture_bank_ < 0)
            return;

        const std::size_t l = static_cast<std::size_t>(capture_layer_);
        const std::size_t b = static_cast<std::size_t>(capture_bank_);

        frames_[l][b] = capture_write_;

        if(capture_write_ > 64)
        {
            if(frames_[l][active_bank_[l]] > 64)
            {
                crossfade_bank_[l] = static_cast<int>(b);
                bank_xfade_[l] = 0.f;
                bank_xfade_inc_[l] = 1.f / (sample_rate_ * 0.45f);
            }
            else
            {
                active_bank_[l] = static_cast<int>(b);
                crossfade_bank_[l] = -1;
                bank_xfade_[l] = 0.f;
            }
        }

        capture_layer_ = -1;
        capture_bank_  = -1;
        capture_write_ = 0;

        // A completed capture must always become audible immediately.
        playing_ = true;
    }

    void CancelCapture()
    {
        if(capture_layer_ >= 0 && capture_bank_ >= 0)
            frames_[capture_layer_][capture_bank_] = 0;

        capture_layer_ = -1;
        capture_bank_  = -1;
        capture_write_ = 0;
    }

    bool IsCapturing() const { return capture_layer_ >= 0; }
    int GetCaptureLayer() const { return capture_layer_; }
    std::size_t GetCaptureFrames() const { return capture_write_; }
    std::size_t GetMaxFrames() const { return max_frames_; }

    bool LayerHasAudio(std::size_t layer) const
    {
        if(layer >= kLayers)
            return false;

        return frames_[layer][active_bank_[layer]] > 64
            || (crossfade_bank_[layer] >= 0
                && frames_[layer][crossfade_bank_[layer]] > 64);
    }

    void SetPlaying(bool value) { playing_ = value; }
    bool IsPlaying() const { return playing_; }

    void SetWindow(float value) { window_ = Clamp01(value); }
    void SetFade(float value)   { fade_ = Clamp01(value); }
    void SetDrift(float value)  { drift_ = Clamp01(value); }

    void SetColor(float value)
    {
        color_ = Clamp01(value);
        UpdateColor();
    }

    void SetSpeed(float value)
    {
        speed_ = Clamp(value, 0.25f, 2.f);
    }

    void SetMaster(float value)
    {
        master_ = Clamp(value, 0.f, 1.2f);
    }

    float GetWindow() const { return window_; }
    float GetFade() const { return fade_; }
    float GetDrift() const { return drift_; }
    float GetColor() const { return color_; }
    float GetSpeed() const { return speed_; }
    float GetMaster() const { return master_; }

    Stereo Process(float input_l, float input_r)
    {
        CaptureSample(input_l, input_r);

        if(!playing_)
            return {0.f, 0.f};

        Stereo sum{0.f, 0.f};
        float active = 0.f;

        for(std::size_t layer = 0; layer < kLayers; ++layer)
        {
            Stereo layer_sig = ProcessLayer(layer);

            if(LayerHasAudio(layer))
            {
                sum.l += layer_sig.l;
                sum.r += layer_sig.r;
                active += 1.f;
            }
        }

        if(active > 1.f)
        {
            const float gain = 1.f / std::sqrt(active);
            sum.l *= gain;
            sum.r *= gain;
        }

        // Global COLOR: darker + gently saturated as it rises.
        lp_l_ += lp_alpha_ * (sum.l - lp_l_);
        lp_r_ += lp_alpha_ * (sum.r - lp_r_);

        const float sat_l = FastSaturate(lp_l_ * drive_);
        const float sat_r = FastSaturate(lp_r_ * drive_);

        sum.l = lp_l_ + (sat_l - lp_l_) * saturation_mix_;
        sum.r = lp_r_ + (sat_r - lp_r_) * saturation_mix_;

        sum.l *= master_;
        sum.r *= master_;

        return {ClampAudio(sum.l), ClampAudio(sum.r)};
    }

  private:
    static float Clamp(float x, float lo, float hi)
    {
        return x < lo ? lo : (x > hi ? hi : x);
    }

    static float Clamp01(float x)
    {
        return Clamp(x, 0.f, 1.f);
    }

    static float ClampAudio(float x)
    {
        return Clamp(x, -1.f, 1.f);
    }

    static int16_t FloatToS16(float x)
    {
        x = ClampAudio(x);
        return static_cast<int16_t>(x * 32767.f);
    }

    static float S16ToFloat(int16_t x)
    {
        return static_cast<float>(x) * (1.f / 32768.f);
    }

    static float FastSaturate(float x)
    {
        x = Clamp(x, -3.f, 3.f);
        const float x2 = x * x;
        return x * (27.f + x2) / (27.f + 9.f * x2);
    }

    static float Smoothstep(float x)
    {
        x = Clamp01(x);
        return x * x * (3.f - 2.f * x);
    }

    float WindowWeight(float phase) const
    {
        phase -= std::floor(phase);
        const float dist_to_edge = std::min(phase, 1.f - phase);

        // fade_ 0: relatively defined repetitions.
        // fade_ 1: very soft, long overlaps.
        const float fade_width = 0.035f + fade_ * 0.465f;
        return Smoothstep(dist_to_edge / fade_width);
    }

    void UpdateColor()
    {
        // COLOR intentionally starts almost neutral.
        lp_alpha_ = Clamp(1.f - color_ * 0.93f, 0.035f, 1.f);
        drive_ = 1.f + color_ * 2.8f;
        saturation_mix_ = color_ * 0.78f;
    }

    void CaptureSample(float l, float r)
    {
        if(capture_layer_ < 0 || capture_bank_ < 0)
            return;

        if(capture_write_ >= max_frames_)
        {
            StopCapture();
            return;
        }

        int16_t* mem = mem_[capture_layer_][capture_bank_];
        if(!mem)
        {
            CancelCapture();
            return;
        }

        mem[capture_write_ * 2]     = FloatToS16(l);
        mem[capture_write_ * 2 + 1] = FloatToS16(r);
        ++capture_write_;
    }

    std::size_t WindowFrames(std::size_t total_frames) const
    {
        if(total_frames < 2)
            return 0;

        const std::size_t min_frames = std::min<std::size_t>(
            total_frames,
            static_cast<std::size_t>(sample_rate_ * 0.060f));

        const std::size_t max_frames = total_frames;

        if(max_frames <= min_frames)
            return max_frames;

        const float shaped = window_ * window_;
        return min_frames
            + static_cast<std::size_t>(
                shaped * static_cast<float>(max_frames - min_frames));
    }

    Stereo ReadInterpolated(std::size_t layer,
                            std::size_t bank,
                            float frame_pos) const
    {
        const std::size_t total = frames_[layer][bank];
        int16_t* mem = mem_[layer][bank];

        if(!mem || total < 2)
            return {0.f, 0.f};

        while(frame_pos < 0.f)
            frame_pos += static_cast<float>(total);

        while(frame_pos >= static_cast<float>(total))
            frame_pos -= static_cast<float>(total);

        const std::size_t i0 = static_cast<std::size_t>(frame_pos);
        const std::size_t i1 = (i0 + 1) % total;
        const float frac = frame_pos - static_cast<float>(i0);

        const float l0 = S16ToFloat(mem[i0 * 2]);
        const float r0 = S16ToFloat(mem[i0 * 2 + 1]);
        const float l1 = S16ToFloat(mem[i1 * 2]);
        const float r1 = S16ToFloat(mem[i1 * 2 + 1]);

        return {
            l0 + (l1 - l0) * frac,
            r0 + (r1 - r0) * frac
        };
    }

    Stereo ProcessBank(std::size_t layer, std::size_t bank)
    {
        const std::size_t total = frames_[layer][bank];
        if(total < 64)
            return {0.f, 0.f};

        const std::size_t win = std::max<std::size_t>(64, WindowFrames(total));
        const float max_start = static_cast<float>(total > win ? total - win : 0);

        float& start = drift_start_[layer][bank];

        // Slow scan through the captured material.
        if(max_start > 1.f && drift_ > 0.0001f)
        {
            const float frames_per_sample = 0.002f + drift_ * drift_ * 0.085f;
            start += frames_per_sample;

            if(start > max_start)
                start -= max_start;
        }
        else
        {
            start = 0.f;
        }

        float& phase = phase_[layer][bank];

        const float p_a = phase;
        float p_b = phase + 0.5f;
        if(p_b >= 1.f)
            p_b -= 1.f;

        const float pos_a = start + p_a * static_cast<float>(win - 1);
        const float pos_b = start + p_b * static_cast<float>(win - 1);

        const Stereo a = ReadInterpolated(layer, bank, pos_a);
        const Stereo b = ReadInterpolated(layer, bank, pos_b);

        const float wa = WindowWeight(p_a);
        const float wb = WindowWeight(p_b);
        const float norm = 1.f / std::max(0.0001f, wa + wb);

        Stereo out{
            (a.l * wa + b.l * wb) * norm,
            (a.r * wa + b.r * wb) * norm
        };

        phase += speed_ / static_cast<float>(win);
        while(phase >= 1.f)
            phase -= 1.f;

        return out;
    }

    Stereo ProcessLayer(std::size_t layer)
    {
        const int active = active_bank_[layer];
        Stereo a = ProcessBank(layer, static_cast<std::size_t>(active));

        const int next = crossfade_bank_[layer];
        if(next < 0)
            return a;

        Stereo b = ProcessBank(layer, static_cast<std::size_t>(next));

        float x = bank_xfade_[layer];
        const float shaped = Smoothstep(x);

        Stereo out{
            a.l + (b.l - a.l) * shaped,
            a.r + (b.r - a.r) * shaped
        };

        x += bank_xfade_inc_[layer];
        if(x >= 1.f)
        {
            active_bank_[layer] = next;
            crossfade_bank_[layer] = -1;
            bank_xfade_[layer] = 0.f;
            bank_xfade_inc_[layer] = 0.f;
        }
        else
        {
            bank_xfade_[layer] = x;
        }

        return out;
    }

    float sample_rate_ = 48000.f;
    std::size_t max_frames_ = 0;

    int16_t* mem_[kLayers][kBanks] = {};
    std::size_t frames_[kLayers][kBanks] = {};
    float phase_[kLayers][kBanks] = {};
    float drift_start_[kLayers][kBanks] = {};

    int active_bank_[kLayers] = {};
    int crossfade_bank_[kLayers] = {};
    float bank_xfade_[kLayers] = {};
    float bank_xfade_inc_[kLayers] = {};

    int capture_layer_ = -1;
    int capture_bank_  = -1;
    std::size_t capture_write_ = 0;

    bool playing_ = true;

    float window_ = 0.34f;
    float fade_   = 0.72f;
    float drift_  = 0.10f;
    float color_  = 0.12f;
    float speed_  = 1.f;
    float master_ = 0.85f;

    float lp_l_ = 0.f;
    float lp_r_ = 0.f;
    float lp_alpha_ = 1.f;
    float drive_ = 1.f;
    float saturation_mix_ = 0.f;
};

} // namespace neoclo
