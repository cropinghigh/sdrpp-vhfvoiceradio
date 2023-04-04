#pragma once

#include <iomanip>

#include <imgui.h>
#include <module.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <signal_path/signal_path.h>
#include <config.h>
#include <dsp/chain.h>
#include <dsp/noise_reduction/noise_blanker.h>
#include <dsp/noise_reduction/fm_if.h>
#include <dsp/noise_reduction/squelch.h>
#include <dsp/multirate/rational_resampler.h>
#include <dsp/filter/deephasis.h>
#include <core.h>
#include <utils/optionlist.h>
#include "radio_interface.h"
#include "demod.h"
#include "dsp/dcs.h"
#include "dsp/ctcss.h"

ConfigManager config;

#define CONCAT(a, b) ((std::string(a) + b).c_str())

std::map<DeemphasisMode, double> deempTaus = {
    { DEEMP_MODE_22US, 22e-6 },
    { DEEMP_MODE_50US, 50e-6 },
    { DEEMP_MODE_75US, 75e-6 }
};

std::map<IFNRPreset, double> ifnrTaps = {
    { IFNR_PRESET_NOAA_APT, 9},
    { IFNR_PRESET_VOICE, 15 },
    { IFNR_PRESET_NARROW_BAND, 31 },
    { IFNR_PRESET_BROADCAST, 32 }
};

class VhfVoiceRadioModule : public ModuleManager::Instance {
public:
    VhfVoiceRadioModule(std::string name) {
        this->name = name;

        // Initialize option lists
        deempModes.define("None", DEEMP_MODE_NONE);
        deempModes.define("22us", DEEMP_MODE_22US);
        deempModes.define("50us", DEEMP_MODE_50US);
        deempModes.define("75us", DEEMP_MODE_75US);

        ifnrPresets.define("NOAA APT", IFNR_PRESET_NOAA_APT);
        ifnrPresets.define("Voice", IFNR_PRESET_VOICE);
        ifnrPresets.define("Narrow Band", IFNR_PRESET_NARROW_BAND);

        // Initialize the config if it doesn't exist
        bool created = false;
        config.acquire();
        if (!config.conf.contains(name)) {
            config.conf[name]["selectedDemodId"] = 0;
            created = true;
        }
        selectedDemodID = config.conf[name]["selectedDemodId"];
        config.release(created);

        // Initialize the VFO
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, 200000, 200000, 50000, 200000, false);
        onUserChangedBandwidthHandler.handler = vfoUserChangedBandwidthHandler;
        onUserChangedBandwidthHandler.ctx = this;
        vfo->wtfVFO->onUserChangedBandwidth.bindHandler(&onUserChangedBandwidthHandler);

        // Initialize IF DSP chain
        ifChainOutputChanged.ctx = this;
        ifChainOutputChanged.handler = ifChainOutputChangeHandler;
        ifChain.init(vfo->output);

        nb.init(NULL, 500.0 / 24000.0, 10.0);
        fmnr.init(NULL, 32);
        squelch.init(NULL, MIN_SQUELCH);

        ifChain.addBlock(&nb, false);
        ifChain.addBlock(&squelch, false);
        ifChain.addBlock(&fmnr, false);

        // Initialize audio DSP chain
        afChain.init(&dummyAudioStream);

        resamp.init(NULL, 250000.0, 48000.0);
        deemp.init(NULL, 50e-6, 48000.0);

        ctcssSquelch.init(NULL, 48000);
        dcsSquelch.init(NULL, 48000);

        afChain.addBlock(&resamp, true);
        afChain.addBlock(&ctcssSquelch, false);
        afChain.addBlock(&dcsSquelch, false);
        afChain.addBlock(&deemp, false);

        // Initialize the sink
        srChangeHandler.ctx = this;
        srChangeHandler.handler = sampleRateChangeHandler;
        stream.init(afChain.out, &srChangeHandler, audioSampleRate);
        sigpath::sinkManager.registerStream(name, &stream);

        // Select the demodulator
        selectDemodByID((DemodID)selectedDemodID);

        // Start IF chain
        ifChain.start();

        // Start AF chain
        afChain.start();

        // Start stream, the rest was started when selecting the demodulator
        stream.start();

        // Register the menu
        gui::menu.registerEntry(name, menuHandler, this, this);

        // Register the module interface
        core::modComManager.registerInterface("vhfvoiceradio", name, moduleInterfaceHandler, this);
    }

    ~VhfVoiceRadioModule() {
        core::modComManager.unregisterInterface(name);
        gui::menu.removeEntry(name);
        stream.stop();
        if (enabled) {
            disable();
        }
        sigpath::sinkManager.unregisterStream(name);
    }

    void postInit() {}

    void enable() {
        enabled = true;
        if (!vfo) {
            vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, 200000, 200000, 50000, 200000, false);
            vfo->wtfVFO->onUserChangedBandwidth.bindHandler(&onUserChangedBandwidthHandler);
        }
        ifChain.setInput(vfo->output, [=](dsp::stream<dsp::complex_t>* out){ ifChainOutputChangeHandler(out, this); });
        ifChain.start();
        selectDemodByID((DemodID)selectedDemodID);
        afChain.start();
    }

    void disable() {
        enabled = false;
        ifChain.stop();
        if (selectedDemod) { selectedDemod->stop(); }
        afChain.stop();
        if (vfo) { sigpath::vfoManager.deleteVFO(vfo); }
        vfo = NULL;
    }

    bool isEnabled() {
        return enabled;
    }

    std::string name;

    enum DemodID {
        RADIO_DEMOD_NFM,
        RADIO_DEMOD_DSD,
        RADIO_DEMOD_OLDDSD,
        _RADIO_DEMOD_COUNT,
    };

private:
    static void menuHandler(void* ctx) {
        VhfVoiceRadioModule* _this = (VhfVoiceRadioModule*)ctx;

        if (!_this->enabled) { style::beginDisabled(); }

        float menuWidth = ImGui::GetContentRegionAvail().x;
        ImGui::BeginGroup();

        ImGui::Columns(2, CONCAT("RadioModeColumns##_", _this->name), false);
        if (ImGui::RadioButton(CONCAT("NFM##_", _this->name), _this->selectedDemodID == 0) && _this->selectedDemodID != 0) {
            _this->selectDemodByID(RADIO_DEMOD_NFM);
        }
        ImGui::NextColumn();
        if (ImGui::RadioButton(CONCAT("DSD##_", _this->name), _this->selectedDemodID == 1) && _this->selectedDemodID != 1) {
            _this->selectDemodByID(RADIO_DEMOD_DSD);
        }
        ImGui::NextColumn();
        if (ImGui::RadioButton(CONCAT("OLD DSD##_", _this->name), _this->selectedDemodID == 2) && _this->selectedDemodID != 2) {
            _this->selectDemodByID(RADIO_DEMOD_OLDDSD);
        }
        ImGui::Columns(1, CONCAT("EndRadioModeColumns##_", _this->name), false);

        ImGui::EndGroup();

        if (!_this->bandwidthLocked) {
            ImGui::LeftLabel("Bandwidth");
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::InputFloat(("##_radio_bw_" + _this->name).c_str(), &_this->bandwidth, 1, 100, "%.0f")) {
                _this->bandwidth = std::clamp<float>(_this->bandwidth, _this->minBandwidth, _this->maxBandwidth);
                _this->setBandwidth(_this->bandwidth);
            }
        }

        // VFO snap interval
        ImGui::LeftLabel("Snap Interval");
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputInt(("##_radio_snap_" + _this->name).c_str(), &_this->snapInterval, 1, 100)) {
            if (_this->snapInterval < 1) { _this->snapInterval = 1; }
            _this->vfo->setSnapInterval(_this->snapInterval);
            config.acquire();
            config.conf[_this->name][_this->selectedDemod->getName()]["snapInterval"] = _this->snapInterval;
            config.release(true);
        }

        // Deemphasis mode
        if (_this->deempAllowed) {
            ImGui::LeftLabel("De-emphasis");
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::Combo(("##_radio_wfm_deemp_" + _this->name).c_str(), &_this->deempId, _this->deempModes.txt)) {
                _this->setDeemphasisMode(_this->deempModes[_this->deempId]);
            }
        }

        // Squelch
        if (ImGui::Checkbox(("Squelch##_radio_sqelch_ena_" + _this->name).c_str(), &_this->squelchEnabled)) {
            _this->setSquelchEnabled(_this->squelchEnabled);
        }
        if (!_this->squelchEnabled && _this->enabled) { style::beginDisabled(); }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::SliderFloat(("##_radio_sqelch_lvl_" + _this->name).c_str(), &_this->squelchLevel, _this->MIN_SQUELCH, _this->MAX_SQUELCH, "%.3fdB")) {
            _this->setSquelchLevel(_this->squelchLevel);
        }
        if (!_this->squelchEnabled && _this->enabled) { style::endDisabled(); }

        // Noise blanker
        if (_this->nbAllowed) {
            if (ImGui::Checkbox(("Noise Blanker##_radio_nb_ena_" + _this->name).c_str(), &_this->nbEnabled)) {
                _this->setNoiseBlankerEnabled(_this->nbEnabled);
            }
            if (!_this->nbEnabled && _this->enabled) { style::beginDisabled(); }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::SliderFloat(("##_radio_nb_lvl_" + _this->name).c_str(), &_this->nbLevel, 0.0f, -100.0f, "%.3fdB")) {
                _this->setNoiseBlankerLevel(_this->nbLevel);
            }
            if (!_this->nbEnabled && _this->enabled) { style::endDisabled(); }
        }

        // FM IF Noise Reduction
        if (_this->FMIFNRAllowed) {
            if (ImGui::Checkbox(("IF Noise Reduction##_radio_fmifnr_ena_" + _this->name).c_str(), &_this->FMIFNREnabled)) {
                _this->setFMIFNREnabled(_this->FMIFNREnabled);
            }
            if (_this->selectedDemodID == RADIO_DEMOD_NFM) {
                if (!_this->FMIFNREnabled && _this->enabled) { style::beginDisabled(); }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
                if (ImGui::Combo(("##_radio_fmifnr_ena_" + _this->name).c_str(), &_this->fmIFPresetId, _this->ifnrPresets.txt)) {
                    _this->setIFNRPreset(_this->ifnrPresets[_this->fmIFPresetId]);
                }
                if (!_this->FMIFNREnabled && _this->enabled) { style::endDisabled(); }
            }
        }


        if(_this->deempAllowed) { //only if it's FM mode
            //CTCSS
            std::stringstream ctcss_freq_stream;
            ctcss_freq_stream << std::fixed << std::setprecision(1) << _this->ctcssSquelch.getCurrentCTCSSFreq();
            if(_this->ctcssSquelch.getCurrentCTCSSActive()) {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), CONCAT("Current tone: ", ctcss_freq_stream.str()));
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), CONCAT("Last       tone: ", ctcss_freq_stream.str()));
            }
            ImGui::SameLine();
            if(ImGui::Button("Copy to CTCSS##_radio_copy_to_ctcss_")) {
                _this->setCtcssFreq(_this->ctcssSquelch.getCurrentCTCSSFreq());
            }
            if(_this->dcsEnabled) { style::beginDisabled(); }
            if (ImGui::Checkbox(("CTCSS##_radio_ctcss_ena_" + _this->name).c_str(), &_this->ctcssEnabled)) {
                _this->setCtcssEnabled(_this->ctcssEnabled);
            }
            if(_this->dcsEnabled) { style::endDisabled(); }
            if (!_this->ctcssEnabled && _this->enabled) { style::beginDisabled(); }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::InputFloat(("##_radio_ctcss_freq_" + _this->name).c_str(), &_this->ctcssFreq, 0.1f, 10.0f, "%.1f")) {
                _this->ctcssFreq = std::clamp<float>(_this->ctcssFreq, 33.0f, 255.0f);
                _this->setCtcssFreq(_this->ctcssFreq);
            }
            if (!_this->ctcssEnabled && _this->enabled) { style::endDisabled(); }

            //DCS
            std::stringstream dcs_code_stream;
            dcs_code_stream << "+" << std::fixed << std::setprecision(1) << _this->dcsSquelch.getCurrentDCSPCode() << "-" << std::fixed << std::setprecision(1) << _this->dcsSquelch.getCurrentDCSNCode();
            if(_this->dcsSquelch.getCurrentDCSActive()) {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), CONCAT("Current code: ", dcs_code_stream.str()));
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), CONCAT("Last       code: ", dcs_code_stream.str()));
            }
            ImGui::SameLine();
            if(ImGui::Button("Copy to DCS##_radio_copy_to_dcs_")) {
                _this->setDcsCode(_this->dcsSquelch.getCurrentDCSPCode());
            }
            if(_this->ctcssEnabled) { style::beginDisabled(); }
            if (ImGui::Checkbox(("DCS##_radio_dcs_ena_" + _this->name).c_str(), &_this->dcsEnabled)) {
                _this->setDcsEnabled(_this->dcsEnabled);
            }
            if(_this->ctcssEnabled) { style::endDisabled(); }
            if (!_this->dcsEnabled && _this->enabled) { style::beginDisabled(); }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::InputInt(("##_radio_dcs_code_" + _this->name).c_str(), &_this->dcsCode, 1, 10)) {
                _this->dcsCode = std::clamp<int>(_this->dcsCode, -754, 754);
                _this->setDcsCode(_this->dcsCode);
            }
            if (!_this->dcsEnabled && _this->enabled) { style::endDisabled(); }
        }

        // Demodulator specific menu
        _this->selectedDemod->showMenu();

        if (!_this->enabled) { style::endDisabled(); }
    }

    demod::Demodulator* instantiateDemod(DemodID id) {
            demod::Demodulator* demod = NULL;
            switch (id) {
                case DemodID::RADIO_DEMOD_NFM:  demod = new demod::NFM(); break;
                case DemodID::RADIO_DEMOD_DSD:  demod = new demod::DSD(); break;
                case DemodID::RADIO_DEMOD_OLDDSD:  demod = new demod::OldDSD(); break;
                default:                        demod = NULL; break;
            }
            if (!demod) { return NULL; }

            // Default config
            double bw = demod->getDefaultBandwidth();
            if (!config.conf[name].contains(demod->getName())) {
                config.conf[name][demod->getName()]["bandwidth"] = bw;
                config.conf[name][demod->getName()]["snapInterval"] = demod->getDefaultSnapInterval();
                config.conf[name][demod->getName()]["squelchLevel"] = MIN_SQUELCH;
                config.conf[name][demod->getName()]["squelchEnabled"] = false;
                config.conf[name][demod->getName()]["ctcssFrequency"] = ctcssFreq;
                config.conf[name][demod->getName()]["ctcssEnabled"] = false;
                config.conf[name][demod->getName()]["dcsCode"] = ctcssFreq;
                config.conf[name][demod->getName()]["dcsEnabled"] = false;
            }
            bw = std::clamp<double>(bw, demod->getMinBandwidth(), demod->getMaxBandwidth());

            // Initialize
            demod->init(name, &config, ifChain.out, bw, stream.getSampleRate());

            return demod;
        }

    void selectDemodByID(DemodID id) {
        auto startTime = std::chrono::high_resolution_clock::now();
        demod::Demodulator* demod = instantiateDemod(id);
        if (!demod) {
            flog::error("Demodulator {0} not implemented", static_cast<int>(id));
            return;
        }
        selectedDemodID = id;
        selectDemod(demod);

        // Save config
        config.acquire();
        config.conf[name]["selectedDemodId"] = id;
        config.release(true);
        auto endTime = std::chrono::high_resolution_clock::now();
        flog::warn("Demod switch took {0} us", (int64_t)((std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime)).count()));
    }

    void selectDemod(demod::Demodulator* demod) {
        // Stop currently selected demodulator and select new
        afChain.setInput(&dummyAudioStream, [=](dsp::stream<dsp::stereo_t>* out){ stream.setInput(out); });
        if (selectedDemod) {
            selectedDemod->stop();
            delete selectedDemod;
        }
        selectedDemod = demod;

        // Give the demodulator the most recent audio SR
        selectedDemod->AFSampRateChanged(audioSampleRate);

        // Set the demodulator's input
        selectedDemod->setInput(ifChain.out);

        // Set AF chain's input
        afChain.setInput(selectedDemod->getOutput(), [=](dsp::stream<dsp::stereo_t>* out){ stream.setInput(out); });

        // Load config
        bandwidth = selectedDemod->getDefaultBandwidth();
        minBandwidth = selectedDemod->getMinBandwidth();
        maxBandwidth = selectedDemod->getMaxBandwidth();
        bandwidthLocked = selectedDemod->getBandwidthLocked();
        snapInterval = selectedDemod->getDefaultSnapInterval();
        squelchLevel = MIN_SQUELCH;
        deempAllowed = selectedDemod->getDeempAllowed();
        deempId = deempModes.valueId((DeemphasisMode)selectedDemod->getDefaultDeemphasisMode());
        squelchEnabled = false;
        postProcEnabled = selectedDemod->getPostProcEnabled();
        FMIFNRAllowed = selectedDemod->getFMIFNRAllowed();
        FMIFNREnabled = false;
        fmIFPresetId = ifnrPresets.valueId(IFNR_PRESET_VOICE);
        nbAllowed = selectedDemod->getNBAllowed();
        nbEnabled = false;
        nbLevel = 0.0f;
        config.acquire();
        if (config.conf[name][selectedDemod->getName()].contains("bandwidth")) {
            bandwidth = config.conf[name][selectedDemod->getName()]["bandwidth"];
            bandwidth = std::clamp<double>(bandwidth, minBandwidth, maxBandwidth);
        }
        if (config.conf[name][selectedDemod->getName()].contains("snapInterval")) {
            snapInterval = config.conf[name][selectedDemod->getName()]["snapInterval"];
        }
        if (config.conf[name][selectedDemod->getName()].contains("squelchLevel")) {
            squelchLevel = config.conf[name][selectedDemod->getName()]["squelchLevel"];
        }
        if (config.conf[name][selectedDemod->getName()].contains("squelchEnabled")) {
            squelchEnabled = config.conf[name][selectedDemod->getName()]["squelchEnabled"];
        }
        if(config.conf[name][selectedDemod->getName()].contains("ctcssFrequency")) {
            ctcssFreq = config.conf[name][selectedDemod->getName()]["ctcssFrequency"];
        }
        if(config.conf[name][selectedDemod->getName()].contains("ctcssEnabled")) {
            ctcssEnabled = config.conf[name][selectedDemod->getName()]["ctcssEnabled"];
        }
        if(config.conf[name][selectedDemod->getName()].contains("dcsCode")) {
            dcsCode = config.conf[name][selectedDemod->getName()]["dcsCode"];
        }
        if(config.conf[name][selectedDemod->getName()].contains("dcsEnabled")) {
            dcsEnabled = config.conf[name][selectedDemod->getName()]["dcsEnabled"];
        }
        if (config.conf[name][selectedDemod->getName()].contains("deempMode")) {
            if (!config.conf[name][selectedDemod->getName()]["deempMode"].is_string()) {
                config.conf[name][selectedDemod->getName()]["deempMode"] = deempModes.key(deempId);
            }

            std::string deempOpt = config.conf[name][selectedDemod->getName()]["deempMode"];
            if (deempModes.keyExists(deempOpt)) {
                deempId = deempModes.keyId(deempOpt);
            }
        }
        if (config.conf[name][selectedDemod->getName()].contains("FMIFNREnabled")) {
            FMIFNREnabled = config.conf[name][selectedDemod->getName()]["FMIFNREnabled"];
        }
        if (config.conf[name][selectedDemod->getName()].contains("fmifnrPreset")) {
            std::string presetOpt = config.conf[name][selectedDemod->getName()]["fmifnrPreset"];
            if (ifnrPresets.keyExists(presetOpt)) {
                fmIFPresetId = ifnrPresets.keyId(presetOpt);
            }
        }
        if (config.conf[name][selectedDemod->getName()].contains("noiseBlankerEnabled")) {
            nbEnabled = config.conf[name][selectedDemod->getName()]["noiseBlankerEnabled"];
        }
        if (config.conf[name][selectedDemod->getName()].contains("noiseBlankerEnabled")) {
            nbEnabled = config.conf[name][selectedDemod->getName()]["noiseBlankerEnabled"];
        }
        if (config.conf[name][selectedDemod->getName()].contains("noiseBlankerLevel")) {
            nbLevel = config.conf[name][selectedDemod->getName()]["noiseBlankerLevel"];
        }
        config.release();

        // Configure VFO
        if (vfo) {
            vfo->setBandwidthLimits(minBandwidth, maxBandwidth, selectedDemod->getBandwidthLocked());
            vfo->setReference(selectedDemod->getVFOReference());
            vfo->setSnapInterval(snapInterval);
            vfo->setSampleRate(selectedDemod->getIFSampleRate(), bandwidth);
        }

        // Configure bandwidth
        setBandwidth(bandwidth);

        // Configure FM IF Noise Reduction
        setIFNRPreset((selectedDemodID == RADIO_DEMOD_NFM) ? ifnrPresets[fmIFPresetId] : IFNR_PRESET_BROADCAST);
        setFMIFNREnabled(FMIFNRAllowed ? FMIFNREnabled : false);

        // Configure squelch
        squelch.setLevel(squelchLevel);
        setSquelchEnabled(squelchEnabled);

        // Configure CTCSS
        setCtcssFreq(ctcssFreq);
        setCtcssEnabled(ctcssEnabled);

        // Configure DCS
        setDcsCode(dcsCode);
        setDcsEnabled(dcsEnabled);

        // Configure noise blanker
        nb.setRate(500.0 / selectedDemod->getIFSampleRate());
        nb.setLevel(nbLevel);
        setNoiseBlankerEnabled(nbEnabled);

        // Configure AF chain
        if (postProcEnabled) {
            // Configure resampler
            afChain.stop();
            resamp.setInSamplerate(selectedDemod->getAFSampleRate());
            setAudioSampleRate(audioSampleRate);
            ctcssSquelch.setInputSr(audioSampleRate);
            dcsSquelch.setInputSr(audioSampleRate);
            afChain.enableBlock(&resamp, [=](dsp::stream<dsp::stereo_t>* out){ printf("rsfin\n"); stream.setInput(out); });
            if(deempAllowed) {
                afChain.setBlockEnabled(&dcsSquelch, true, [=](dsp::stream<dsp::stereo_t>* out){ stream.setInput(out); });
                afChain.setBlockEnabled(&ctcssSquelch, true, [=](dsp::stream<dsp::stereo_t>* out){ stream.setInput(out); });
            } else {
                afChain.setBlockEnabled(&dcsSquelch, false, [=](dsp::stream<dsp::stereo_t>* out){ stream.setInput(out); });
                afChain.setBlockEnabled(&ctcssSquelch, false, [=](dsp::stream<dsp::stereo_t>* out){ stream.setInput(out); });
            }

            // Configure deemphasis
            setDeemphasisMode(deempModes[deempId]);
        }
        else {
            // Disable everything if post processing is disabled
            afChain.disableAllBlocks([=](dsp::stream<dsp::stereo_t>* out){ stream.setInput(out); });
        }

        // Start new demodulator
        selectedDemod->start();
    }


    void setBandwidth(double bw) {
        bw = std::clamp<double>(bw, minBandwidth, maxBandwidth);
        bandwidth = bw;
        if (!selectedDemod) { return; }
        vfo->setBandwidth(bandwidth);
        selectedDemod->setBandwidth(bandwidth);

        config.acquire();
        config.conf[name][selectedDemod->getName()]["bandwidth"] = bandwidth;
        config.release(true);
    }

    void setAudioSampleRate(double sr) {
        audioSampleRate = sr;
        if (!selectedDemod || !enabled) { return; }
        selectedDemod->AFSampRateChanged(audioSampleRate);
        if (!postProcEnabled && vfo) {
            // If postproc is disabled, IF SR = AF SR
            minBandwidth = selectedDemod->getMinBandwidth();
            maxBandwidth = selectedDemod->getMaxBandwidth();
            bandwidth = selectedDemod->getIFSampleRate();
            vfo->setBandwidthLimits(minBandwidth, maxBandwidth, selectedDemod->getBandwidthLocked());
            vfo->setSampleRate(selectedDemod->getIFSampleRate(), bandwidth);
            return;
        }

        afChain.stop();

        // Configure resampler
        resamp.setOutSamplerate(audioSampleRate);
        ctcssSquelch.setInputSr(audioSampleRate);
        dcsSquelch.setInputSr(audioSampleRate);

        // Configure deemphasis sample rate
        deemp.setSamplerate(audioSampleRate);

        afChain.start();
    }

    void setDeemphasisMode(DeemphasisMode mode) {
        deempId = deempModes.valueId(mode);
        if (!postProcEnabled || !selectedDemod) { return; }
        bool deempEnabled = (mode != DEEMP_MODE_NONE);
        if (deempEnabled) { deemp.setTau(deempTaus[mode]); }
        afChain.setBlockEnabled(&deemp, deempEnabled, [=](dsp::stream<dsp::stereo_t>* out){ stream.setInput(out); });

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["deempMode"] = deempModes.key(deempId);
        config.release(true);
    }

    void setSquelchEnabled(bool enable) {
        squelchEnabled = enable;
        if (!selectedDemod) { return; }
        ifChain.setBlockEnabled(&squelch, squelchEnabled, [=](dsp::stream<dsp::complex_t>* out){ selectedDemod->setInput(out); });

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["squelchEnabled"] = squelchEnabled;
        config.release(true);
    }

    void setSquelchLevel(float level) {
        squelchLevel = std::clamp<float>(level, MIN_SQUELCH, MAX_SQUELCH);
        squelch.setLevel(squelchLevel);

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["squelchLevel"] = squelchLevel;
        config.release(true);
    }

    void setCtcssEnabled(bool enable) {
        ctcssEnabled = enable;
        ctcssSquelch.setSquelchEnabled(ctcssEnabled);

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["ctcssEnabled"] = ctcssEnabled;
        config.release(true);
    }

    void setCtcssFreq(float freq) {
        ctcssFreq = freq;
        ctcssSquelch.setSquelchFrequency(ctcssFreq);

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["ctcssFrequency"] = ctcssFreq;
        config.release(true);
    }

    void setDcsEnabled(bool enable) {
        dcsEnabled = enable;
        dcsSquelch.setSquelchEnabled(dcsEnabled);

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["dcsEnabled"] = dcsEnabled;
        config.release(true);
    }

    void setDcsCode(int code) {
        dcsCode = code;
        dcsSquelch.setSquelchCode(dcsCode);

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["dcsCode"] = dcsCode;
        config.release(true);
    }

    void setFMIFNREnabled(bool enabled) {
        FMIFNREnabled = enabled;
        if (!selectedDemod) { return; }
        ifChain.setBlockEnabled(&fmnr, FMIFNREnabled, [=](dsp::stream<dsp::complex_t>* out){ selectedDemod->setInput(out); });

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["FMIFNREnabled"] = FMIFNREnabled;
        config.release(true);
    }

    void setNoiseBlankerEnabled(bool enabled) {
        nbEnabled = enabled;
        if (!selectedDemod) { return; }
        ifChain.setBlockEnabled(&nb, nbEnabled, [=](dsp::stream<dsp::complex_t>* out){ selectedDemod->setInput(out); });

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["noiseBlankerEnabled"] = nbEnabled;
        config.release(true);
    }

    void setNoiseBlankerLevel(float level) {
        nbLevel = level;
        if (!selectedDemod) { return; }
        nb.setLevel(nbLevel);

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["noiseBlankerLevel"] = nbLevel;
        config.release(true);
    }

    void setIFNRPreset(IFNRPreset preset) {
        // Don't save if in broadcast mode
        if (preset == IFNR_PRESET_BROADCAST) {
            if (!selectedDemod) { return; }
            fmnr.setBins(ifnrTaps[preset]);
            return;
        }

        fmIFPresetId = ifnrPresets.valueId(preset);
        if (!selectedDemod) { return; }
        fmnr.setBins(ifnrTaps[preset]);

        // Save config
        config.acquire();
        config.conf[name][selectedDemod->getName()]["fmifnrPreset"] = ifnrPresets.key(fmIFPresetId);
        config.release(true);
    }

    static void vfoUserChangedBandwidthHandler(double newBw, void* ctx) {
        VhfVoiceRadioModule* _this = (VhfVoiceRadioModule*)ctx;
        _this->setBandwidth(newBw);
    }

    static void sampleRateChangeHandler(float sampleRate, void* ctx) {
        VhfVoiceRadioModule* _this = (VhfVoiceRadioModule*)ctx;
        _this->setAudioSampleRate(sampleRate);
    }

    static void ifChainOutputChangeHandler(dsp::stream<dsp::complex_t>* output, void* ctx) {
        VhfVoiceRadioModule* _this = (VhfVoiceRadioModule*)ctx;
        if (!_this->selectedDemod) { return; }
        _this->selectedDemod->setInput(output);
    }

    static void moduleInterfaceHandler(int code, void* in, void* out, void* ctx) {
        VhfVoiceRadioModule* _this = (VhfVoiceRadioModule*)ctx;
        if (!_this->enabled || !_this->selectedDemod) { return; }

        // Execute commands
        if(out) {
            switch(code) {
                case VHFVORADIO_IFACE_CMD_GET_MODE: {
                    int* _out = (int*)out;
                    *_out = _this->selectedDemodID;
                    break;
                }
                case VHFVORADIO_IFACE_CMD_GET_BANDWIDTH: {
                    float* _out = (float*)out;
                    *_out = _this->bandwidth;
                    break;
                }
                case VHFVORADIO_IFACE_CMD_GET_SQUELCH_ENABLED: {
                    bool* _out = (bool*)out;
                    *_out = _this->squelchEnabled;
                    break;
                }
                case VHFVORADIO_IFACE_CMD_GET_SQUELCH_LEVEL: {
                    float* _out = (float*)out;
                    *_out = _this->squelchLevel;
                    break;
                }
                case VHFVORADIO_IFACE_CMD_GET_CTCSS_ENABLED: {
                    bool* _out = (bool*)out;
                    *_out = _this->ctcssEnabled;
                    break;
                }
                case VHFVORADIO_IFACE_CMD_GET_CTCSS_TONE: {
                    float* _out = (float*)out;
                    *_out = _this->ctcssFreq;
                    break;
                }
                case VHFVORADIO_IFACE_CMD_GET_DCS_ENABLED: {
                    bool* _out = (bool*)out;
                    *_out = _this->dcsEnabled;
                    break;
                }
                case VHFVORADIO_IFACE_CMD_GET_DCS_CODE: {
                    int* _out = (int*)out;
                    *_out = _this->dcsCode;
                    break;
                }
                default:
                    break;
            }
        } else if(in) {
            switch(code) {
                case VHFVORADIO_IFACE_CMD_SET_MODE: {
                    int* _in = (int*)in;
                    _this->selectDemodByID((DemodID)*_in);
                    break;
                }
                case VHFVORADIO_IFACE_CMD_SET_BANDWIDTH: {
                    float* _in = (float*)in;
                    if (_this->bandwidthLocked) { break; }
                    _this->setBandwidth(*_in);
                    break;
                }
                case VHFVORADIO_IFACE_CMD_SET_SQUELCH_ENABLED: {
                    bool* _in = (bool*)in;
                    _this->setSquelchEnabled(*_in);
                    break;
                }
                case VHFVORADIO_IFACE_CMD_SET_SQUELCH_LEVEL: {
                    float* _in = (float*)in;
                    _this->setSquelchLevel(*_in);
                    break;
                }
                case VHFVORADIO_IFACE_CMD_SET_CTCSS_ENABLED: {
                    bool* _in = (bool*)in;
                    _this->setCtcssEnabled(*_in);
                    break;
                }
                case VHFVORADIO_IFACE_CMD_SET_CTCSS_TONE: {
                    float* _in = (float*)in;
                    _this->setCtcssFreq(*_in);
                    break;
                }
                case VHFVORADIO_IFACE_CMD_SET_DCS_ENABLED: {
                    bool* _in = (bool*)in;
                    _this->setDcsEnabled(*_in);
                    break;
                }
                case VHFVORADIO_IFACE_CMD_SET_DCS_CODE: {
                    int* _in = (int*)in;
                    _this->setDcsCode(*_in);
                    break;
                }
                default:
                    break;
            }
        }

        // Success
        return;
    }

    // Handlers
    EventHandler<double> onUserChangedBandwidthHandler;
    EventHandler<float> srChangeHandler;
    EventHandler<dsp::stream<dsp::complex_t>*> ifChainOutputChanged;
    EventHandler<dsp::stream<dsp::stereo_t>*> afChainOutputChanged;

    VFOManager::VFO* vfo = NULL;

    // IF chain
    dsp::chain<dsp::complex_t> ifChain;
    dsp::noise_reduction::FMIF fmnr;
    dsp::noise_reduction::Squelch squelch;
    dsp::noise_reduction::NoiseBlanker nb;

    // Audio chain
    dsp::stream<dsp::stereo_t> dummyAudioStream;
    dsp::chain<dsp::stereo_t> afChain;
    dsp::multirate::RationalResampler<dsp::stereo_t> resamp;
    dsp::CTCSSSquelch ctcssSquelch;
    dsp::DCSSquelch dcsSquelch;
    dsp::filter::Deemphasis<dsp::stereo_t> deemp;


    SinkManager::Stream stream;

    demod::Demodulator* selectedDemod = NULL;

    OptionList<std::string, DeemphasisMode> deempModes;
    OptionList<std::string, IFNRPreset> ifnrPresets;

    double audioSampleRate = 48000.0;
    float minBandwidth;
    float maxBandwidth;
    float bandwidth;
    bool bandwidthLocked;
    int snapInterval;
    int selectedDemodID = 1;
    bool postProcEnabled;

    bool squelchEnabled = false;
    float squelchLevel;

    bool ctcssEnabled = false;
    float ctcssFreq = 88.5f;

    bool dcsEnabled = false;
    int dcsCode = +25;

    int deempId = 0;
    bool deempAllowed;

    bool FMIFNRAllowed;
    bool FMIFNREnabled = false;
    int fmIFPresetId;

    bool notchEnabled = false;
    float notchPos = 0;
    float notchWidth = 500;

    bool nbAllowed;
    bool nbEnabled;
    float nbLevel = -100.0f;

    const double MIN_SQUELCH = -100.0;
    const double MAX_SQUELCH = 0.0;

    bool enabled = true;
};
