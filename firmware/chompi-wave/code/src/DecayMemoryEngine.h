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
        reverse_         = false;
        lp_l_            = 0.f;
        lp_r_            = 0.f;
        flutter_phase_   = 0.f;
        flutter_drift_   = 0.f;
        drift_counter_   = 0;
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

    void SetOutputGain(float v)
    {
        output_gain_ = Clamp(v, 0.f, 1.5f);
    }

    float GetAge() const { return age_; }
    float GetMutation() const { return mutation_; }
    float GetCharacter() const { return character_; }
    float GetMemory() const { return memory_; }
    float GetOutputGain() const { return output_gain_; }

    void SetSliceCount(std::size_t count)
    {
        count = std::max<std::size_t>(1, std::min<std::size_t>(25, count));
        if(count == slice_count_)
            return;

        slice_count_ = count;
        if(HasAudio())
            ConfigureSlice(current_slice_ % slice_count_, reverse_, slice_rate_);
    }

    std::size_t GetSliceCount() const { return slice_count_; }
    std::size_t GetRecordedFrames() const { return recorded_frames_; }
    std::size_t GetMaxFrames() const { return max_frames_; }
    std::size_t GetCurrentSlice() const { return current_slice_; }

    void TriggerSlice(std::size_t slice_index, bool reverse = false, float rate = 1.f)
    {
        if(!HasAudio() || capturing_)
            return;

        playing_ = true;
        ConfigureSlice(slice_index, reverse, rate);
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

        const Stereo source   = ReadInterpolated(read_pos_);
        const Stereo degraded = Degrade(source);

        if(!decay_hold_ && write_amount_ > 0.f)
            WriteBack(read_pos_, degraded, write_amount_);

        const float fade = SliceFadeGain();
        Stereo out{degraded.l * fade * output_gain_,
                   degraded.r * fade * output_gain_};

        flutter_phase_ += flutter_rate_hz_ / sample_rate_;
        if(flutter_phase_ >= 1.f)
            flutter_phase_ -= 1.f;

        const float tri = 1.f - 4.f * std::fabs(flutter_phase_ - 0.5f);

        if(++drift_counter_ >= 64)
        {
            drift_counter_ = 0;
            flutter_drift_ += RandomSigned() * flutter_drift_step_;
            flutter_drift_ *= 0.94f;
            flutter_drift_ = Clamp(flutter_drift_, -0.012f, 0.012f);
        }

        const float rate_mod = 1.f + tri * flutter_depth_ + flutter_drift_;
        const float step     = std::max(0.05f, slice_rate_ * rate_mod);
        read_pos_ += reverse_ ? -step : step;

        return out;
    }

  private:
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
        const float damage = age_ * (0.25f + 0.75f * character_);

        lp_alpha_ = Clamp(1.f - 0.92f * damage, 0.035f, 1.f);

        drive_          = 1.f + age_ * (0.35f + 2.65f * character_);
        saturation_mix_ = age_ * (0.18f + 0.72f * character_);
        dust_amount_    = age_ * age_ * character_ * 0.0045f;

        write_amount_ = age_ * (0.0015f + 0.075f * memory_);

        flutter_depth_      = age_ * character_ * 0.0045f;
        flutter_rate_hz_    = 0.16f + 0.55f * character_;
        flutter_drift_step_ = age_ * character_ * 0.00020f;
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
        if(!HasAudio() || slice_count_ == 0)
            return 0;
        return std::max<std::size_t>(1, recorded_frames_ / slice_count_);
    }

    void ConfigureSlice(std::size_t slice_index, bool reverse, float rate)
    {
        if(!HasAudio())
            return;

        current_slice_ = slice_index % slice_count_;
        const std::size_t frames_per_slice = SliceFrames();

        slice_start_ = current_slice_ * frames_per_slice;
        slice_end_   = current_slice_ == slice_count_ - 1
                           ? recorded_frames_
                           : std::min(recorded_frames_, slice_start_ + frames_per_slice);

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
        std::size_t next = (current_slice_ + 1) % slice_count_;

        if(Random01() < m)
        {
            if(Random01() < (0.88f - 0.38f * m))
            {
                int offset = static_cast<int>(NextRandom() % 5u) - 2;
                int candidate = static_cast<int>(current_slice_) + offset;
                while(candidate < 0)
                    candidate += static_cast<int>(slice_count_);
                next = static_cast<std::size_t>(candidate) % slice_count_;
            }
            else
            {
                next = static_cast<std::size_t>(NextRandom() % static_cast<uint32_t>(slice_count_));
            }
        }

        if(Random01() < m * m * 0.42f)
            next = current_slice_;

        const bool reverse = Random01() < m * 0.28f;

        float rate = 1.f;
        if(Random01() < m * 0.30f)
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

        const std::size_t fade_frames = std::min<std::size_t>(128, std::max<std::size_t>(8, len / 10));
        const float local = reverse_ ? static_cast<float>(slice_end_ - 1) - read_pos_
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
        const float frac     = frame_pos - static_cast<float>(i0);

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
        amount     = Clamp01(amount);

        const std::size_t i0 = static_cast<std::size_t>(frame_pos);
        const std::size_t i1 = std::min(recorded_frames_ - 1, i0 + 1);
        const float frac     = frame_pos - static_cast<float>(i0);

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

        mem_[frame * 2] = FloatToS16(old_l + (target.l - old_l) * amount);
        mem_[frame * 2 + 1] = FloatToS16(old_r + (target.r - old_r) * amount);
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

    float sample_rate_ = 48000.f;
    int16_t* mem_ = nullptr;
    std::size_t max_frames_ = 0;
    std::size_t recorded_frames_ = 0;
    std::size_t write_frame_ = 0;

    bool capturing_  = false;
    bool playing_    = false;
    bool decay_hold_ = false;

    std::size_t slice_count_   = 25;
    std::size_t current_slice_ = 0;
    std::size_t slice_start_   = 0;
    std::size_t slice_end_     = 0;

    float read_pos_   = 0.f;
    float slice_rate_ = 1.f;
    bool reverse_     = false;

    float mutation_    = 0.25f;
    float age_         = 0.20f;
    float character_   = 0.35f;
    float memory_      = 0.45f;
    float output_gain_ = 0.90f;

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

    uint32_t rng_ = 0x4E454F43u;
};

} // namespace neoclo
