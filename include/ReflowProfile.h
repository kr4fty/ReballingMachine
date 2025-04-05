#ifndef REFLOW_PROFILE_H
#define REFLOW_PROFILE_H

typedef struct {
    int times[20];
    int temperatures[20];
    int profileLength;
} ReflowProfile;

typedef struct {
    const char* name;
    ReflowProfile profile;
} ProfileEntry;

typedef struct {
    ProfileEntry profiles[10];
    int currentProfileIndex;
    int profileCount;
} ReflowHeatingProfileController;

void ReflowHeatingProfileController_init(ReflowHeatingProfileController* controller) {
    controller->currentProfileIndex = -1;
    controller->profileCount = 0;
}

void ReflowHeatingProfileController_addProfile(ReflowHeatingProfileController* controller, const char* name, ReflowProfile profile) {
    if (controller->profileCount < 10) {
        controller->profiles[controller->profileCount].name = name;
        controller->profiles[controller->profileCount].profile = profile;
        controller->profileCount++;
    }
}

void ReflowHeatingProfileController_selectProfileByName(ReflowHeatingProfileController* controller, const char* name) {
    for (int i = 0; i < controller->profileCount; i++) {
        if (strcmp(controller->profiles[i].name, name) == 0) {
            controller->currentProfileIndex = i;
            return;
        }
    }
    controller->currentProfileIndex = -1;
}

void ReflowHeatingProfileController_selectProfile(ReflowHeatingProfileController* controller, uint8_t selected) {
    if (selected >= 0 && selected <= controller->profileCount) {
        controller->currentProfileIndex = selected;
        return;
    }
    controller->currentProfileIndex = -1;
}

int ReflowHeatingProfileController_getTemperature(ReflowHeatingProfileController* controller, int time) {
    if (controller->currentProfileIndex == -1) {
        return -1; // No profile selected
    }

    ReflowProfile profile = controller->profiles[controller->currentProfileIndex].profile;
    for (int i = 0; i < profile.profileLength; i++) {
        if (profile.times[i] == time) {
            return profile.temperatures[i];
        }
    }
    return -1; // Time not found
}

int ReflowHeatingProfileController_getTime(ReflowHeatingProfileController* controller, int index) {
    if (controller->currentProfileIndex == -1 || index < 0 || index >= controller->profiles[controller->currentProfileIndex].profile.profileLength) {
        return -1; // Invalid index
    }
    return controller->profiles[controller->currentProfileIndex].profile.times[index];
}

int ReflowHeatingProfileController_getTemperatureForIndex(ReflowHeatingProfileController* controller, int index) {
    if (controller->currentProfileIndex == -1 || index < 0 || index >= controller->profiles[controller->currentProfileIndex].profile.profileLength) {
        return -1; // Invalid index
    }
    return controller->profiles[controller->currentProfileIndex].profile.temperatures[index];
}

int ReflowHeatingProfileController_getProfileLength(ReflowHeatingProfileController* controller) {
    if (controller->currentProfileIndex == -1) {
        return 0; // No profile selected
    }
    return controller->profiles[controller->currentProfileIndex].profile.profileLength;
}

int ReflowHeatingProfileController_getProfileCount(ReflowHeatingProfileController* controller) {
    return controller->profileCount;
}

const char* ReflowHeatingProfileController_getProfileName(ReflowHeatingProfileController* controller, int index)
{
    if (index < 0 || index >= controller->profileCount) {
        return nullptr; // Invalid index
    }
    return controller->profiles[index].name;
}

const char* ReflowHeatingProfileController_getSelectedProfileName(ReflowHeatingProfileController* controller)
{
    if (controller->currentProfileIndex == -1) {
        return nullptr; // No profile selected
    }
    return controller->profiles[controller->currentProfileIndex].name;
}

#endif
