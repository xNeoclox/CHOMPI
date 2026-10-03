#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace neoclo
{

class DecayMemoryEngine
{
  public:
    struct Stereo
    {
        float l;
        float r;
    };

    void Init(float sample_rate, int16_t* interleaved_stereo_memory, std::size_t max_frames)
    {
        sample_rate_ = sample_rate > 1000.f ? sample_rate : 48000.f;
        mem_         = interleaved_stereo_memory;
        max_frames_  = max_frames;
        Reset();
        UpdateDecayCoefficients();
        UpdateGlitchCoefficients();
    }

    void Reset()
    {
        recorded_frames_ = 0;
        write_frame_     = 0;
        capturing_       = false;
        playing_         = false;
        decay_hold_      = false;
        current_slice_   = 0;
        slice_start_     = 0;
        slice_end_       = 0;
        read_pos_        = 0.f;
        slice_rate_      = 1.f;
        playback_speed_  = 1.f;
        reverse_         = false;
        lp_l_            = 0.f;
        lp_r_            = 0.f;
        flutter_phase_   = 0.f;
        flutter_drift_   = 0.f;
        drift_counter_   = 0;
        crush_counter_   = 0;
        crush_hold_l_    = 0.f;
        crush_hold_r_    = 0.f;
        rng_             = 0x4E454F43u;
    }

    void StartCapture()
    {
        if(!mem_ || max_frames_ < 2)
            return;

        recorded_frames_ = 0;
        write_frame_     = 0;
        capturing_       = true;
        playing_         = false;
        current_slice_   = 0;
        lp_l_            = 0.f;
        lp_r_            = 0.f;
    }

    void StopCapture()
    {
        capturing_ = false;

        if(recorded_frames_ >= 2)
        {
            playing_       = true;
            current_slice_ = 0;
            ConfigureSlice(0, false, 1.f);
        }
    }

    bool IsCapturing() const { return capturing_; }
    bool HasAudio() const { return recorded_frames_ > 1; }

    void SetPlaying(bool playing) { playing_ = playing && HasAudio(); }
    bool IsPlaying() const { return playing_; }

    void SetDecayHold(bool hold) { decay_hold_ = hold; }
    bool GetDecayHold() const { return decay_hold_; }

    void SetAge(float v)
    {
        age_ = Clamp01(v);
        UpdateDecayCoefficients();
    }

    void SetMutation(float v) { mutation_ = Clamp01(v); }

    void SetCharacter(float v)
    {
        character_ = Clamp01(v);
        UpdateDecayCoefficients();
    }

    void SetMemory(float v)
    {
        memory_ = Clamp01(v);
        UpdateDecayCoefficients();
    }

    void SetSliceSize(float v)
    {
        slice_size_ = Clamp01(v);
        if(HasAudio())
            ConfigureSlice(current_slice_, reverse_, slice_rate_);
    }

    float GetSliceSize() const { return slice_size_; }

    void SetPlaybackSpeed(float speed)
    {
        playback_speed_ = Clamp(speed, 0.25f, 2.f);
    }

    float GetPlaybackSpeed() const { return playback_speed_; }

    void SetGlitch(float amount)
    {
        glitch_ = Clamp01(amount);
        UpdateGlitchCoefficients();
    }

    float GetGlitch() const { return glitch_; }

    void SetOutputGain(float v)
    {
        output_gain_ = Clamp(v, 0.f, 1.5f);
    }

    float GetAge() const { return age_; }
    float GetMutation() const { return mutation_; }
    float GetCharacter() const { return character_; }
    float GetMemory() const { return memory_; }
    float GetOutputGain() const { return output_gain_; }

    std::size_t GetRecordedFrames() const { return recorded_frames_; }
    std::size_t GetMaxFrames() const { return max_frames_; }
    std::size_t GetCurrentSlice() const { return current_slice_; }

    void TriggerSlice(std::size_t slice_index, bool reverse = false, float rate = 1.f)
    {
        if(!HasAudio() || capturing_)
            return;

        playing_ = true;
        ConfigureSlice(slice_index % kSliceCount, reverse, rate);
    }

    Stereo Process(float in_l, float in_r)
    {
        if(capturing_)
        {
            if(mem_ && write_frame_ < max_frames_)
            {
                mem_[write_frame_ * 2]     = FloatToS16(in_l);
                mem_[write_frame_ * 2 + 1] = FloatToS16(in_r);
                ++write_frame_;
                recorded_frames_ = write_frame_;
                if(write_frame_ >= max_frames_)
                    StopCapture();
            }
            else
            {
                StopCapture();
            }

            return {in_l * output_gain_, in_r * output_gain_};
        }

        if(!playing_ || !HasAudio())
            return {0.f, 0.f};

        if(SliceFinished())
            TriggerAutoSlice();

        const Stereo source = ReadInterpolated(read_pos_);
        const Stereo aged   = Degrade(source);

        if(!decay_hold_ && write_amount_ > 0.f)
            WriteBack(read_pos_, aged, write_amount_);

        const Stereo effected = ApplyGlitch(aged);

        const float fade = SliceFadeGain();
        Stereo out{effected.l * fade * output_gain_,
                   effected.r * fade * output_gain_};

        flutter_phase_ += flutter_rate_hz_ / sample_rate_;
        if(flutter_phase_ >= 1.f)
            flutter_phase_ -= 1.f;

        const float tri = 1.f - 4.f * std::fabs(flutter_phase_ - 0.5f);

        if(++drift_counter_ >= 64)
        {
            drift_counter_ = 0;
            flutter_drift_ += RandomSigned() * flutter_drift_step_;
            flutter_drift_ *= 0.94f;
            flutter_drift_ = Clamp(flutter_drift_, -0.016f, 0.016f);
        }

        const float rate_mod = 1.f + tri * flutter_depth_ + flutter_drift_;
        const float step = std::max(0.025f, slice_rate_ * playback_speed_ * rate_mod);
        read_pos_ += reverse_ ? -step : step;

        return out;
    }

  private:
    static constexpr std::size_t kSliceCount = 25;

    static float Clamp(float x, float lo, float hi)
    {
        return x < lo ? lo : (x > hi ? hi : x);
    }

    static float Clamp01(float x) { return Clamp(x, 0.f, 1.f); }
    static float ClampAudio(float x) { return Clamp(x, -1.f, 1.f); }

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

    void UpdateDecayCoefficients()
    {
        const float a = age_;
        const float c = character_;
        const float a2 = a * a;

        // Much stronger than v0.2: at high AGE the feedback copy gets dark quickly.
        lp_alpha_ = Clamp(1.f - 0.92f * a - 0.05f * a2 * c, 0.028f, 1.f);

        drive_          = 1.f + a * (0.65f + 3.35f * c);
        saturation_mix_ = a * (0.20f + 0.72f * c);
        dust_amount_    = a2 * (0.0006f + 0.008f * c);

        // Destructive write-back. Each revisit changes the stored generation.
        write_amount_ = a * (0.025f + 0.24f * memory_);

        flutter_depth_      = a * c * 0.0065f;
        flutter_rate_hz_    = 0.13f + 0.70f * c;
        flutter_drift_step_ = a * c * 0.00030f;
    }

    void UpdateGlitchCoefficients()
    {
        const float g2 = glitch_ * glitch_;
        crush_hold_samples_ = 1 + static_cast<uint32_t>(g2 * 23.f);

        // 16-bit at zero, down to about 5-bit at maximum.
        const int bits = std::max(5, 16 - static_cast<int>(glitch_ * 11.f));
        crush_levels_ = static_cast<float>(1u << (bits - 1));
    }

    uint32_t NextRandom()
    {
        uint32_t x = rng_;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        rng_ = x;
        return x;
    }

    float Random01()
    {
        return static_cast<float>(NextRandom() & 0x00FFFFFFu) * (1.f / 16777215.f);
    }

    float RandomSigned() { return Random01() * 2.f - 1.f; }

    std::size_t SliceFrames() const
    {
        if(!HasAudio())
            return 0;

        const std::size_t min_frames = std::min<std::size_t>(
            recorded_frames_, static_cast<std::size_t>(sample_rate_ * 0.030f));
        const std::size_t max_frames = std::min<std::size_t>(
            recorded_frames_, static_cast<std::size_t>(sample_rate_ * 2.50f));

        if(max_frames <= min_frames)
            return recorded_frames_;

        const float shaped = slice_size_ * slice_size_;
        const std::size_t result = min_frames
            + static_cast<std::size_t>(shaped * static_cast<float>(max_frames - min_frames));

        return std::max<std::size_t>(2, std::min(recorded_frames_, result));
    }

    void ConfigureSlice(std::size_t slice_index, bool reverse, float rate)
    {
        if(!HasAudio())
            return;

        current_slice_ = slice_index % kSliceCount;
        const std::size_t len = SliceFrames();

        const std::size_t max_start = recorded_frames_ > len ? recorded_frames_ - len : 0;
        if(kSliceCount > 1)
            slice_start_ = (current_slice_ * max_start) / (kSliceCount - 1);
        else
            slice_start_ = 0;

        slice_end_ = std::min(recorded_frames_, slice_start_ + len);
        if(slice_end_ <= slice_start_ + 1)
            slice_end_ = std::min(recorded_frames_, slice_start_ + 2);

        reverse_    = reverse;
        slice_rate_ = Clamp(std::fabs(rate), 0.25f, 4.f);
        read_pos_   = reverse_ ? static_cast<float>(slice_end_ - 1)
                               : static_cast<float>(slice_start_);
    }

    void TriggerAutoSlice()
    {
        if(!HasAudio())
            return;

        const float m = mutation_;
        std::size_t next = (current_slice_ + 1) % kSliceCount;

        if(Random01() < m)
        {
            if(Random01() < (0.88f - 0.38f * m))
            {
                int offset = static_cast<int>(NextRandom() % 5u) - 2;
                int candidate = static_cast<int>(current_slice_) + offset;
                while(candidate < 0)
                    candidate += static_cast<int>(kSliceCount);
                next = static_cast<std::size_t>(candidate) % kSliceCount;
            }
            else
            {
                next = static_cast<std::size_t>(NextRandom() % static_cast<uint32_t>(kSliceCount));
            }
        }

        if(Random01() < m * m * 0.42f)
            next = current_slice_;

        const bool reverse = Random01() < m * 0.30f;

        float rate = 1.f;
        if(Random01() < m * 0.34f)
        {
            static const int semitones[] = {-12, -7, -5, 0, 5, 7, 12};
            const int st = semitones[NextRandom() % 7u];
            rate = std::pow(2.f, static_cast<float>(st) / 12.f);
        }

        ConfigureSlice(next, reverse, rate);
    }

    bool SliceFinished() const
    {
        if(slice_end_ <= slice_start_ + 1)
            return true;

        return reverse_ ? read_pos_ <= static_cast<float>(slice_start_)
                        : read_pos_ >= static_cast<float>(slice_end_ - 1);
    }

    float SliceFadeGain() const
    {
        const std::size_t len = slice_end_ > slice_start_ ? slice_end_ - slice_start_ : 0;
        if(len < 8)
            return 1.f;

        const std::size_t fade_frames = std::min<std::size_t>(
            192, std::max<std::size_t>(8, len / 10));

        const float local = reverse_
            ? static_cast<float>(slice_end_ - 1) - read_pos_
            : read_pos_ - static_cast<float>(slice_start_);
        const float remaining = static_cast<float>(len - 1) - local;

        float gain = 1.f;
        if(local < static_cast<float>(fade_frames))
            gain = std::min(gain, local / static_cast<float>(fade_frames));
        if(remaining < static_cast<float>(fade_frames))
            gain = std::min(gain, remaining / static_cast<float>(fade_frames));

        return Clamp01(gain);
    }

    Stereo ReadInterpolated(float frame_pos) const
    {
        if(!mem_ || !HasAudio())
            return {0.f, 0.f};

        frame_pos = Clamp(frame_pos, 0.f, static_cast<float>(recorded_frames_ - 1));
        const std::size_t i0 = static_cast<std::size_t>(frame_pos);
        const std::size_t i1 = std::min(recorded_frames_ - 1, i0 + 1);
        const float frac = frame_pos - static_cast<float>(i0);

        const float l0 = S16ToFloat(mem_[i0 * 2]);
        const float r0 = S16ToFloat(mem_[i0 * 2 + 1]);
        const float l1 = S16ToFloat(mem_[i1 * 2]);
        const float r1 = S16ToFloat(mem_[i1 * 2 + 1]);

        return {l0 + (l1 - l0) * frac,
                r0 + (r1 - r0) * frac};
    }

    void WriteBack(float frame_pos, const Stereo& target, float amount)
    {
        if(!mem_ || !HasAudio() || amount <= 0.f)
            return;

        frame_pos = Clamp(frame_pos, 0.f, static_cast<float>(recorded_frames_ - 1));
        amount = Clamp01(amount);

        const std::size_t i0 = static_cast<std::size_t>(frame_pos);
        const std::size_t i1 = std::min(recorded_frames_ - 1, i0 + 1);
        const float frac = frame_pos - static_cast<float>(i0);

        BlendFrame(i0, target, amount * (1.f - frac));
        if(i1 != i0)
            BlendFrame(i1, target, amount * frac);
    }

    void BlendFrame(std::size_t frame, const Stereo& target, float amount)
    {
        if(amount <= 0.f)
            return;

        const float old_l = S16ToFloat(mem_[frame * 2]);
        const float old_r = S16ToFloat(mem_[frame * 2 + 1]);

        // Tiny loss per generation is part of AGE: old memories slowly disappear.
        const float loss = 1.f - age_ * 0.006f;
        const float next_l = (old_l + (target.l - old_l) * amount) * loss;
        const float next_r = (old_r + (target.r - old_r) * amount) * loss;

        mem_[frame * 2]     = FloatToS16(next_l);
        mem_[frame * 2 + 1] = FloatToS16(next_r);
    }

    Stereo Degrade(const Stereo& in)
    {
        lp_l_ += lp_alpha_ * (in.l - lp_l_);
        lp_r_ += lp_alpha_ * (in.r - lp_r_);

        float l = lp_l_;
        float r = lp_r_;

        if(saturation_mix_ > 0.0001f)
        {
            const float sat_l = FastSaturate(l * drive_);
            const float sat_r = FastSaturate(r * drive_);
            l += (sat_l - l) * saturation_mix_;
            r += (sat_r - r) * saturation_mix_;
        }

        if(dust_amount_ > 0.000001f)
        {
            l += RandomSigned() * dust_amount_;
            r += RandomSigned() * dust_amount_;
        }

        return {ClampAudio(l), ClampAudio(r)};
    }

    Stereo ApplyGlitch(const Stereo& in)
    {
        if(glitch_ < 0.001f)
            return in;

        if(crush_counter_ == 0)
        {
            const float ql = std::round(in.l * crush_levels_) / crush_levels_;
            const float qr = std::round(in.r * crush_levels_) / crush_levels_;
            crush_hold_l_ = ql;
            crush_hold_r_ = qr;
            crush_counter_ = crush_hold_samples_;
        }

        --crush_counter_;

        const float mix = glitch_;
        return {
            ClampAudio(in.l + (crush_hold_l_ - in.l) * mix),
            ClampAudio(in.r + (crush_hold_r_ - in.r) * mix)
        };
    }

    float sample_rate_ = 48000.f;
    int16_t* mem_ = nullptr;
    std::size_t max_frames_ = 0;
    std::size_t recorded_frames_ = 0;
    std::size_t write_frame_ = 0;

    bool capturing_  = false;
    bool playing_    = false;
    bool decay_hold_ = false;

    std::size_t current_slice_ = 0;
    std::size_t slice_start_   = 0;
    std::size_t slice_end_     = 0;

    float read_pos_        = 0.f;
    float slice_rate_      = 1.f;
    float playback_speed_  = 1.f;
    bool reverse_          = false;

    float slice_size_   = 0.32f;
    float mutation_     = 0.25f;
    float age_          = 0.10f;
    float character_    = 0.50f;
    float memory_       = 0.65f;
    float glitch_       = 0.f;
    float output_gain_  = 0.90f;

    float lp_alpha_           = 1.f;
    float drive_              = 1.f;
    float saturation_mix_     = 0.f;
    float dust_amount_        = 0.f;
    float write_amount_       = 0.f;
    float flutter_depth_      = 0.f;
    float flutter_rate_hz_    = 0.2f;
    float flutter_drift_step_ = 0.f;

    float lp_l_          = 0.f;
    float lp_r_          = 0.f;
    float flutter_phase_ = 0.f;
    float flutter_drift_ = 0.f;
    uint32_t drift_counter_ = 0;

    uint32_t crush_hold_samples_ = 1;
    uint32_t crush_counter_      = 0;
    float crush_levels_          = 32768.f;
    float crush_hold_l_          = 0.f;
    float crush_hold_r_          = 0.f;

    uint32_t rng_ = 0x4E454F43u;
};

} // namespace neoclo
