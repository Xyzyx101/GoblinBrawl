#include "stdafx.h"
#include "Sound.h"


Sound::Sound() {}
Sound *Sound::instance = nullptr;

Sound::~Sound() {
	if( m_audEngine ) {
		m_audEngine->Suspend();
	}
	m_musicLoop.reset();
	m_lavaLoop.reset();
}


bool Sound::Init() {
	if( instance!=nullptr ) {
		return false;
	}
	instance = new Sound;

	AUDIO_ENGINE_FLAGS eflags = AudioEngine_Default;
#ifdef _DEBUG
	eflags = eflags|AudioEngine_Debug;
#endif
	instance->m_audEngine.reset( new AudioEngine( eflags ) );
	instance->m_retryAudio = false;

	instance->m_getHit.reset( new SoundEffect( instance->m_audEngine.get(), L"Sound/GoblinHit.wav" ) );
	instance->m_attack.reset( new SoundEffect( instance->m_audEngine.get(), L"Sound/GoblinLaugh.wav" ) );
	instance->m_fire.reset( new SoundEffect( instance->m_audEngine.get(), L"Sound/Fire.wav" ) );
	instance->m_lava.reset( new SoundEffect( instance->m_audEngine.get(), L"Sound/Lava.wav" ) );
	instance->m_weaponHit.reset( new SoundEffect( instance->m_audEngine.get(), L"Sound/WeaponHit.wav" ) );
	instance->m_jump.reset( new SoundEffect( instance->m_audEngine.get(), L"Sound/GoblinJump.wav" ) );
	instance->m_step.reset( new SoundEffect( instance->m_audEngine.get(), L"Sound/GoblinStep1.wav" ) );
	instance->m_ambient.reset( new SoundEffect( instance->m_audEngine.get(), L"Sound/GameMusic.wav" ) );
	instance->m_musicLoop = instance->m_ambient->CreateInstance();
	instance->m_lavaLoop = instance->m_lava->CreateInstance();



	return true;
}

void Sound::Play( SOUNDS sound ) {



	instance->m_audEngine->Suspend();
	instance->m_audEngine->Resume();

	if( instance->m_retryAudio ) {
		instance->m_retryAudio = false;

	} else if( !instance->m_audEngine->Update() ) {
		if( instance->m_audEngine->IsCriticalError() ) {
			instance->m_retryAudio = true;
		}
	}


	switch( sound ) {
	case SOUNDS::FIRE:
		instance->m_fire->Play();
		break;
	case SOUNDS::LAVA:
		instance->m_musicLoop->Play( true );
		break;
	case SOUNDS::ATTACKING:
		instance->m_attack->Play();
		break;
	case SOUNDS::GETHIT:
		instance->m_getHit->Play();
		break;
	case SOUNDS::STEP:
		instance->m_step->Play();
		break;
	case SOUNDS::JUMPING:
		instance->m_jump->Play();
		break;
	case SOUNDS::WEAPONHIT:
		instance->m_weaponHit->Play();
		break;
	case SOUNDS::MUSIC:
		instance->m_musicLoop->Play( true );
		break;
	default:
		// Code
		break;
	}
}

