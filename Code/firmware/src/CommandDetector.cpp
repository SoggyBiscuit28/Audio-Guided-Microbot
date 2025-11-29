#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "I2SSampler.h"
#include "AudioProcessor.h"
#include "NeuralNetwork.h"
#include "RingBuffer.h"
#include "CommandDetector.h"
#include "CommandProcessor.h"

#define WINDOW_SIZE 320
#define STEP_SIZE 160
#define POOLING_SIZE 6
#define AUDIO_LENGTH 16000

namespace
{
    constexpr float kCommandProbabilityThresholds[NUMBER_COMMANDS] = {
        0.70f, // forward
        0.71f, // backward
        0.50f, // left
        0.48f, // right (slightly lower)
        0.50f  // unknown/silence slot stays strict
    };
    constexpr float kCommandProbabilityMargins[NUMBER_COMMANDS] = {
        0.35f,
        0.37f,
        0.18f,
        0.15f,
        0.20f
    };
    constexpr float kSilenceScore = 1e-6f;
    constexpr float kProbabilityThreshold = 0.55f;
    constexpr float kProbabilityMargin = 0.20f;
    constexpr float kProbabilityThresholdStrong = 0.45f;
    constexpr float kProbabilityMarginStrong = 0.15f;
    constexpr unsigned long kDetectionCooldownMs = 1000;
    constexpr unsigned long kDebugLogIntervalMs = 200;

    constexpr float kSpeechGateSnrThreshold = 1.25f;
    constexpr float kSpeechGateStrongSnr = 2.15f;
    constexpr float kSpeechGateMinRms = 58.5f;
    constexpr float kSpeechGateStrongRms = 106.5f;
    constexpr float kSpeechGateMinAbsDelta = 6.5f;
    constexpr float kSpeechGateMaxZeroCross = 0.28f;
    constexpr float kSpeechGateZcrMargin = 0.08f;
    constexpr int kSpeechGateMinHits = 2;
    constexpr int kSpeechGateHangoverHits = 4;
    constexpr unsigned long kSpeechGateReleaseMs = 600;

    struct GateMetrics
    {
        float average_abs;
        float rms;
        float zero_cross_rate;
    };

    GateMetrics compute_gate_metrics(RingBufferAccessor *reader)
    {
        const int start_index = reader->getIndex();
        double sum_abs = 0.0;
        double sum_sq = 0.0;
        int zero_crossings = 0;
        int16_t previous_sample = reader->getCurrentSample();
        for (int i = 0; i < AUDIO_LENGTH; ++i)
        {
            const int16_t sample = reader->getCurrentSample();
            sum_abs += fabsf(static_cast<float>(sample));
            sum_sq += static_cast<double>(sample) * sample;
            if ((sample > 0 && previous_sample <= 0) || (sample < 0 && previous_sample >= 0))
            {
                zero_crossings++;
            }
            previous_sample = sample;
            reader->moveToNextSample();
        }
        reader->setIndex(start_index);

        GateMetrics metrics;
        metrics.average_abs = sum_abs / AUDIO_LENGTH;
        metrics.rms = sqrtf(sum_sq / AUDIO_LENGTH);
        metrics.zero_cross_rate = static_cast<float>(zero_crossings) / AUDIO_LENGTH;
        return metrics;
    }
}

CommandDetector::CommandDetector(I2SSampler *sample_provider, CommandProcessor *command_procesor)
{
    m_last_detection = 0;
    m_scores_index = 0;
    m_command_processor = command_procesor;
    // save the sample provider for use later
    m_sample_provider = sample_provider;
    // some stats on performance
    m_average_detect_time = 0;
    m_number_of_runs = 0;
    // Create our neural network
    m_nn = new NeuralNetwork();
    Serial.println("Created Neral Net");
    // create our audio processor
    m_audio_processor = new AudioProcessor(AUDIO_LENGTH, WINDOW_SIZE, STEP_SIZE, POOLING_SIZE);
    // clear down the window
    for (int i = 0; i < COMMAND_WINDOW; i++)
    {
        for (int j = 0; j < NUMBER_COMMANDS; j++)
        {
            m_scores[i][j] = 0;
        }
    }
    m_scores_index = 0;
    m_gate_noise_estimate = 0;
    m_gate_noise_zcr = 0;
    m_gate_consecutive_hits = 0;
    m_gate_release_deadline = 0;
    m_debug_last_average_abs = 0;
    m_debug_last_rms = 0;
    m_debug_last_zero_cross_rate = 0;
    m_debug_last_zcr_limit = 0;
    m_debug_last_snr = 0;
    m_debug_last_noise_estimate = 0;
    m_debug_last_noise_zcr = 0;
    m_debug_last_hits = 0;
    m_debug_last_energy_trigger = false;
    m_debug_last_strong_energy = false;
    m_debug_last_zcr_ok = false;
    m_debug_last_gate_open = false;
    m_debug_last_best_prob = 0;
    m_debug_last_second_prob = 0;
    m_debug_last_prob_margin = 0;
    m_debug_last_best_index = NUMBER_COMMANDS - 1;
    m_debug_last_accept_command = false;
    m_last_debug_log_ms = 0;

    Serial.println("Created audio processor");
}

CommandDetector::~CommandDetector()
{
    delete m_nn;
    m_nn = NULL;
    delete m_audio_processor;
    m_audio_processor = NULL;
    uint32_t free_ram = esp_get_free_heap_size();
    Serial.printf("Free ram after DetectWakeWord cleanup %d\n", free_ram);
}

void CommandDetector::run()
{
    // time how long this takes for stats
    long start = millis();
    // get access to the samples that have been read in
    RingBufferAccessor *reader = m_sample_provider->getRingBufferReader();
    // rewind by 1 second
    reader->rewind(AUDIO_LENGTH);
    bool gate_open = shouldRunInference(reader, start);
    bool is_valid = false;
    if (gate_open)
    {
        // get hold of the input buffer for the neural network so we can feed it data
        float *input_buffer = m_nn->getInputBuffer();
        // process the samples to get the spectrogram
        is_valid = m_audio_processor->get_spectrogram(reader, input_buffer);
    }
    // finished with the sample reader
    delete reader;
    if (gate_open && is_valid)
    {
        // get the prediction for the spectrogram
        m_nn->predict();
        cachePredictionRow(m_nn->getOutputBuffer());
    }
    else
    {
        cachePredictionRow(nullptr);
    }
    // get the best score
    float scores[NUMBER_COMMANDS] = {0, 0, 0, 0, 0};
    for (int i = 0; i < COMMAND_WINDOW; i++)
    {
        for (int j = 0; j < NUMBER_COMMANDS; j++)
        {
            scores[j] += m_scores[i][j];
        }
    }
    float best_score = scores[0];
    int best_index = 0;
    float second_score = -std::numeric_limits<float>::infinity();
    int second_index = -1;
    for (int i = 1; i < NUMBER_COMMANDS; i++)
    {
        if (scores[i] > best_score)
        {
            second_score = best_score;
            second_index = best_index;
            best_index = i;
            best_score = scores[i];
        }
        else if (scores[i] > second_score)
        {
            second_score = scores[i];
            second_index = i;
        }
    }
    if (second_index < 0)
    {
        second_index = best_index;
        second_score = scores[best_index];
    }

    float max_logit = scores[0];
    for (int i = 1; i < NUMBER_COMMANDS; ++i)
    {
        max_logit = std::max(max_logit, scores[i]);
    }
    float denom = 0;
    float best_exp = 0;
    float second_exp = 0;
    for (int i = 0; i < NUMBER_COMMANDS; ++i)
    {
        float value = expf(scores[i] - max_logit);
        denom += value;
        if (i == best_index)
        {
            best_exp = value;
        }
        if (i == second_index)
        {
            second_exp = value;
        }
    }
    float best_prob = (denom > 0) ? best_exp / denom : 0.0f;
    float second_prob = (denom > 0) ? second_exp / denom : 0.0f;
    float prob_margin = best_prob - second_prob;
    long end = millis();

    const bool strong_gate = m_debug_last_strong_energy;
    float base_prob_threshold = kProbabilityThreshold;
    float base_margin_threshold = kProbabilityMargin;
    if (best_index >= 0 && best_index < NUMBER_COMMANDS)
    {
        base_prob_threshold = kCommandProbabilityThresholds[best_index];
        base_margin_threshold = kCommandProbabilityMargins[best_index];
    }
    const float prob_threshold = strong_gate ? std::min(base_prob_threshold, kProbabilityThresholdStrong) : base_prob_threshold;
    const float margin_threshold = strong_gate ? std::min(base_margin_threshold, kProbabilityMarginStrong) : base_margin_threshold;
    bool accept_command = best_index != NUMBER_COMMANDS - 1 &&
                          best_prob >= prob_threshold &&
                          prob_margin >= margin_threshold &&
                          (start - m_last_detection) > kDetectionCooldownMs;
    m_debug_last_best_prob = best_prob;
    m_debug_last_second_prob = second_prob;
    m_debug_last_prob_margin = prob_margin;
    m_debug_last_best_index = best_index;
    m_debug_last_accept_command = accept_command;

    if ((start - m_last_debug_log_ms) >= kDebugLogIntervalMs)
    {
        Serial.printf("[Gate] avg_abs=%.1f rms=%.1f zcr=%.3f limit=%.3f snr=%.2f noise=%.1f zcr_noise=%.3f hits=%d/%d energy=%d strong=%d zcr_ok=%d open=%d\n",
                      m_debug_last_average_abs,
                      m_debug_last_rms,
                      m_debug_last_zero_cross_rate,
                  m_debug_last_zcr_limit,
                      m_debug_last_snr,
                      m_debug_last_noise_estimate,
                  m_debug_last_noise_zcr,
                      m_debug_last_hits,
                      kSpeechGateMinHits,
                      m_debug_last_energy_trigger,
                  m_debug_last_strong_energy,
                      m_debug_last_zcr_ok,
                      gate_open);
        if (gate_open)
        {
            Serial.printf("[NN] valid=%d best=%d best_prob=%.2f second=%.2f margin=%.2f th=%.2f marg_th=%.2f accept=%d cooldown_ok=%d\n",
                          is_valid,
                          best_index,
                          best_prob,
                          second_prob,
                          prob_margin,
                          prob_threshold,
                          margin_threshold,
                          accept_command,
                          ((start - m_last_detection) > kDetectionCooldownMs));
        }
        else
        {
            Serial.println("[NN] Skipped inference: gate closed");
        }
        m_last_debug_log_ms = start;
    }
    // sanity check best score and check the cool down period
    if (accept_command)
    {
        m_last_detection = start;
        m_command_processor->queueCommand(best_index, best_prob);
    }
    // compute the stats
    m_average_detect_time = (end - start) * 0.1 + m_average_detect_time * 0.9;
    m_number_of_runs++;
    // log out some timing info
    if (m_number_of_runs == 100)
    {
        m_number_of_runs = 0;
        Serial.printf("Average detection time %.fms\n", m_average_detect_time);
    }
}

bool CommandDetector::shouldRunInference(RingBufferAccessor *reader, unsigned long now)
{
    GateMetrics metrics = compute_gate_metrics(reader);
    if (m_gate_noise_estimate <= 0.0f)
    {
        m_gate_noise_estimate = std::max(metrics.average_abs, 1.0f);
    }
    if (m_gate_noise_zcr <= 0.0f)
    {
        m_gate_noise_zcr = std::max(metrics.zero_cross_rate, 0.001f);
    }
    const bool looks_like_noise = metrics.average_abs <= m_gate_noise_estimate * 1.3f;
    const float alpha = looks_like_noise ? 0.05f : 0.005f;
    const float zcr_alpha = looks_like_noise ? 0.1f : 0.01f;
    m_gate_noise_estimate = (1.0f - alpha) * m_gate_noise_estimate + alpha * metrics.average_abs;
    m_gate_noise_zcr = (1.0f - zcr_alpha) * m_gate_noise_zcr + zcr_alpha * metrics.zero_cross_rate;

    const float snr = (metrics.average_abs + 1.0f) / (m_gate_noise_estimate + 1.0f);
    const bool abs_triggered = metrics.average_abs >= (m_gate_noise_estimate + kSpeechGateMinAbsDelta);
    const bool energy_triggered = (snr >= kSpeechGateSnrThreshold) ||
                                  (metrics.rms >= kSpeechGateMinRms) ||
                                  abs_triggered;
    const bool strong_energy = (snr >= kSpeechGateStrongSnr) || (metrics.rms >= kSpeechGateStrongRms);
    const float adaptive_zcr_limit = std::min(kSpeechGateMaxZeroCross, m_gate_noise_zcr + kSpeechGateZcrMargin);
    const bool zcr_ok = metrics.zero_cross_rate <= adaptive_zcr_limit;

    if (energy_triggered && zcr_ok)
    {
        if (strong_energy)
        {
            if (m_gate_consecutive_hits < kSpeechGateMinHits)
            {
                m_gate_consecutive_hits = kSpeechGateMinHits;
            }
        }
        else if (m_gate_consecutive_hits < kSpeechGateHangoverHits)
        {
            m_gate_consecutive_hits++;
        }
        m_gate_release_deadline = now + kSpeechGateReleaseMs;
    }
    else if (now > m_gate_release_deadline)
    {
        m_gate_consecutive_hits = 0;
    }
    const bool gate_open = m_gate_consecutive_hits >= kSpeechGateMinHits;
    m_debug_last_average_abs = metrics.average_abs;
    m_debug_last_rms = metrics.rms;
    m_debug_last_zero_cross_rate = metrics.zero_cross_rate;
    m_debug_last_snr = snr;
    m_debug_last_noise_estimate = m_gate_noise_estimate;
    m_debug_last_noise_zcr = m_gate_noise_zcr;
    m_debug_last_zcr_limit = adaptive_zcr_limit;
    m_debug_last_hits = m_gate_consecutive_hits;
    m_debug_last_energy_trigger = energy_triggered;
    m_debug_last_strong_energy = strong_energy;
    m_debug_last_zcr_ok = zcr_ok;
    m_debug_last_gate_open = gate_open;
    return gate_open;
}

void CommandDetector::cachePredictionRow(const float *predictions)
{
    for (int i = 0; i < NUMBER_COMMANDS; ++i)
    {
        const float value = (predictions != nullptr) ? std::max(predictions[i], kSilenceScore) : kSilenceScore;
        m_scores[m_scores_index][i] = log(value);
    }
    m_scores_index = (m_scores_index + 1) % COMMAND_WINDOW;
}