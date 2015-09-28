#pragma once

#include <vector>
#include <memory>
#include "DirectX_11_1_Includes.h"
#include "GameTimer.h"
#include "Camera.h"
#include "Floor.h"
#include "Lava.h"
#include "Walls.h"
#include "FirePlinth.h"
#include "Goblin.h"
#include "Lighting.h"
#include "Keyboard.h"
#include "GamePad.h"
#include "Audio.h"
#include <random>


enum SOUNDS {
	FIRE,
	LAVA,
	STEP,
	ATTACKING,
	GETHIT,
	WEAPONHIT,
	JUMPING,
	MUSIC

};


class Sound {
public:
	~Sound();
	static bool Init();
	static void Play( SOUNDS sound );
private:
	Sound();
	static Sound*instance;
	std::unique_ptr<DirectX::AudioEngine> m_audEngine;
	bool m_retryAudio;
	std::unique_ptr<DirectX::SoundEffect> m_getHit;
	std::unique_ptr<DirectX::SoundEffect> m_fire;
	std::unique_ptr<DirectX::SoundEffect> m_lava;
	std::unique_ptr<DirectX::SoundEffect> m_attack;
	std::unique_ptr<DirectX::SoundEffect> m_ambient;
	std::unique_ptr<DirectX::SoundEffect> m_weaponHit;
	std::unique_ptr<DirectX::SoundEffect> m_jump;
	std::unique_ptr<DirectX::SoundEffect> m_step;
	std::unique_ptr<std::mt19937> m_random;
	std::unique_ptr<DirectX::SoundEffectInstance> m_musicLoop;
	std::unique_ptr<DirectX::SoundEffectInstance> m_lavaLoop;

};

