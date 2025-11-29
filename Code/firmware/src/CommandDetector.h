#ifndef _detect_wake_word_state_h_
#define _detect_wake_word_state_h_

class I2SSampler;
class NeuralNetwork;
class AudioProcessor;
class CommandProcessor;
class RingBufferAccessor;

#define NUMBER_COMMANDS 5
#define COMMAND_WINDOW 3

class CommandDetector
{
private:
    CommandProcessor *m_command_processor;
    I2SSampler *m_sample_provider;
    NeuralNetwork *m_nn;
    AudioProcessor *m_audio_processor;
    float m_average_detect_time;
    int m_number_of_runs;
    float m_scores[COMMAND_WINDOW][NUMBER_COMMANDS];
    int m_scores_index;
    unsigned long m_last_detection;
    float m_gate_noise_estimate;
    float m_gate_noise_zcr;
    int m_gate_consecutive_hits;
    unsigned long m_gate_release_deadline;
    // Debug metrics to help trace gating/inference decisions.
    float m_debug_last_average_abs;
    float m_debug_last_rms;
    float m_debug_last_zero_cross_rate;
    float m_debug_last_zcr_limit;
    float m_debug_last_snr;
    float m_debug_last_noise_estimate;
    float m_debug_last_noise_zcr;
    int m_debug_last_hits;
    bool m_debug_last_energy_trigger;
    bool m_debug_last_strong_energy;
    bool m_debug_last_zcr_ok;
    bool m_debug_last_gate_open;
    float m_debug_last_best_prob;
    float m_debug_last_second_prob;
    float m_debug_last_prob_margin;
    int m_debug_last_best_index;
    bool m_debug_last_accept_command;
    unsigned long m_last_debug_log_ms;

public:
    CommandDetector(I2SSampler *sample_provider, CommandProcessor *command_processor);
    ~CommandDetector();
    void run();

private:
    bool shouldRunInference(RingBufferAccessor *reader, unsigned long now);
    void cachePredictionRow(const float *predictions);
};

extern int flag;

#endif