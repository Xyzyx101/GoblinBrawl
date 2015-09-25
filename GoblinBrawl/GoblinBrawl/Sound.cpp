#include "stdafx.h"
#include "Sound.h"


Sound::Sound() {
	instance = this;
}


Sound::~Sound() {}

bool Sound::Init() {
	instance->
}

void Sound::Play(SOUNDS sound) {
	instance->internalPlay( sound );
}