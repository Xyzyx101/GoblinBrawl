#pragma once

enum SOUNDS {
	FIRE,
	LAVA,
	WALK
};

class Sound {
public:
	~Sound();
	static bool Init();
	static void Play(SOUNDS sound);
private:
	Sound();
	static Sound*	instance;
};


