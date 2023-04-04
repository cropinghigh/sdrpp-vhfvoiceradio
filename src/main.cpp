#include "radio_module.h"

SDRPP_MOD_INFO{
    /* Name:            */ "vhfvoiceradio",
    /* Description:     */ "Modified radio decoder for V/UHF voice",
    /* Author:          */ "cropinghigh",
    /* Version:         */ 0, 0, 6,
    /* Max instances    */ -1
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    config.setPath(core::args["root"].s() + "/vhfvoiceradio_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new VhfVoiceRadioModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (VhfVoiceRadioModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
